import gradio as gr
import os
import time
import platform
import psutil
import pandas as pd
import math
import shutil
import re
import subprocess
import GPUtil
from gradio_modal import Modal
from src.gui.machine_info import get_system_info, estimate_generation_time
from src.gui.model_generation import generate_model

# --- Configuration ---
ASSETS_DIR = "./assets"
RESULTS_DIR = "./results"
GRADIO_TEMP_DIR = "./gui_tmp"
os.makedirs(GRADIO_TEMP_DIR, exist_ok=True)
os.environ["GRADIO_TEMP_DIR"] = GRADIO_TEMP_DIR

os.makedirs(ASSETS_DIR, exist_ok=True)
os.makedirs(RESULTS_DIR, exist_ok=True)

# --- CSS for Fixed Height Preview Box ---
css = """
.fixed-height-preview {
    height: 400px;
    min-height: 400px;
    overflow-y: auto;
    border: 1px solid #cccccc;
    padding: 8px;
    display: flex;
    flex-direction: column;
    justify-content: center;
    margin-bottom: 10px;
}
.fixed-height-preview > * {
   max-width: 100%;
}
.fixed-height-preview > .status-message {
    margin-top: auto;
    text-align: center;
    padding-top: 5px;
    margin-left: auto;
    margin-right: auto;
}
.visualized-file {
    text-align: center;
}
.preview-col {
    margin: 0 5px;
}
.modal-content {
    padding: 20px;
}
"""

# --- Helper Functions ---

def get_asset_folder_choices():
    """Generates the list of choices for the asset folder dropdown, only subfolders."""
    try:
        os.makedirs(ASSETS_DIR, exist_ok=True)
        asset_subfolders = [
            d for d in os.listdir(ASSETS_DIR)
            if os.path.isdir(os.path.join(ASSETS_DIR, d)) and
               not d.startswith('.') and d != "__pycache__"
        ]
        return sorted(asset_subfolders)
    except Exception as e:
        print(f"Error listing asset subfolders for dropdown: {e}")
        return ["Error listing folders"]

def visualize_file(selected_path: str | None):
    """Determines file type and returns Gradio update objects for visualization components."""
    img_update = gr.update(visible=False, value=None)
    vid_update = gr.update(visible=False, value=None)
    mod_update = gr.update(visible=False, value=None)
    status_msg = "Select a file to preview." # Default message

    if not selected_path:
        return img_update, vid_update, mod_update, gr.update(value=status_msg)

    if not isinstance(selected_path, str):
         print(f"Warning: visualize_file expected string path, got {type(selected_path)}. Value: {selected_path}")
         status_msg = "Internal error: Invalid selection type."
         return img_update, vid_update, mod_update, gr.update(value=status_msg)

    if not os.path.exists(selected_path):
        status_msg = f"Selected path does not exist: {os.path.basename(selected_path)}"
        return img_update, vid_update, mod_update, gr.update(value=status_msg)

    if os.path.isdir(selected_path):
         status_msg = f"Selected item is a directory: {os.path.basename(selected_path)}"
         return img_update, vid_update, mod_update, gr.update(value=status_msg)

    file_ext = os.path.splitext(selected_path)[1].lower()
    filename = os.path.basename(selected_path)

    image_exts = {'.png', '.jpg', '.jpeg', '.bmp', '.gif', '.webp'}
    video_exts = {'.mp4', '.avi', '.mov', '.mkv', '.webm'}
    
    # Models that Gradio can typically preview directly
    viewable_model_exts = {'.obj', '.glb', '.gltf', '.stl', '.ply'}
    # Models that we want to show a special message for
    special_message_model_exts = {'.fbx', '.abc'}

    try:
        if file_ext in image_exts:
            img_update = gr.update(value=selected_path, visible=True)
            status_msg = f"Preview: {filename}"
        elif file_ext in video_exts:
            vid_update = gr.update(value=selected_path, visible=True)
            status_msg = f"Preview: {filename}"
        elif file_ext in viewable_model_exts:
            mod_update = gr.update(value=selected_path, clear_color=[0.0, 0.0, 0.0, 0.0], visible=True)
            status_msg = f"Preview: {filename}"
        elif file_ext in special_message_model_exts:
            # For FBX and ABC, set a specific message and ensure model viewer is not shown
            status_msg = f"{filename}: Preview not supported for this format. Please use dedicated 3D software to view."
            mod_update = gr.update(value=None, visible=False) # Ensure it's hidden
        else:
            status_msg = f"Unsupported file type: {filename}"
            print(f"Unsupported file type selected: {selected_path}")
        
    except Exception as e:
        status_msg = f"Error displaying file: {filename}\n{e}"
        print(f"Error displaying file {selected_path}: {e}")
        # Ensure all previews are hidden on error
        img_update = gr.update(visible=False, value=None)
        vid_update = gr.update(visible=False, value=None)
        mod_update = gr.update(visible=False, value=None)

    # Return the determined status message
    return img_update, vid_update, mod_update, gr.update(value=status_msg)

def trigger_folder_upload_modal(uploaded_items_list: list[str] | None):
    """Triggered by UploadButton. Stores temp paths and shows the modal."""
    print(f"Upload button clicked. Received items: {uploaded_items_list}")
    if not uploaded_items_list:
        return (gr.update(value="No items selected for upload."),
                gr.update(value=None),
                gr.update(visible=False),
                gr.update(value=""))

    status_msg = f"Selected {len(uploaded_items_list)} item(s). Please enter a folder name to upload them into."
    global temp_files
    temp_files = uploaded_items_list.copy()
    return (gr.update(value=status_msg),
            gr.update(value=uploaded_items_list),
            gr.update(visible=True),
            gr.update(value=""))

def copy_uploaded_items_to_subfolder(temp_item_paths: list[str] | None, new_folder_name: str, current_asset_folder_value: str | None):
    """Copies items from temp paths into a new subfolder in ASSETS_DIR after verifying required files."""
    updates = [
        gr.update(value="Processing upload..."),
        gr.update(root_dir=ASSETS_DIR, file_count="single", label=f"Assets Explorer (`{ASSETS_DIR}/`)"),
        gr.update(choices=get_asset_folder_choices(), interactive=False),
        gr.update(value=None),
        gr.update(visible=False)
    ]
    yield updates

    temp_item_paths = temp_files.copy() if temp_item_paths is None else temp_item_paths
    if not temp_item_paths:
        updates = [
            gr.update(value="Error: No items found in temporary storage for upload."),
            gr.update(root_dir=ASSETS_DIR, file_count="single", label=f"Assets Explorer (`{ASSETS_DIR}/`)"),
            gr.update(choices=get_asset_folder_choices(), interactive=True),
            gr.update(value=None),
            gr.update(visible=False)
        ]
        yield updates
        return

    if not new_folder_name or not new_folder_name.strip():
        updates = [
            gr.update(value="Error: Folder name cannot be empty."),
            gr.update(root_dir=ASSETS_DIR, file_count="single", label=f"Assets Explorer (`{ASSETS_DIR}/`)"),
            gr.update(choices=get_asset_folder_choices(), interactive=True),
            gr.update(value=None),
            gr.update(visible=False)
        ]
        yield updates
        return

    if re.search(r'[\\/:*?"<>|.\x00-\x1F]', new_folder_name): # Added period to invalid characters
        updates = [
            gr.update(value="Error: Folder name contains invalid characters (e.g., \\ / : * ? \" < > | .)."),
            gr.update(root_dir=ASSETS_DIR, file_count="single", label=f"Assets Explorer (`{ASSETS_DIR}/`)"),
            gr.update(choices=get_asset_folder_choices(), interactive=True),
            gr.update(value=None),
            gr.update(visible=False)
        ]
        yield updates
        return

    destination_dir = os.path.join(ASSETS_DIR, new_folder_name)

    if os.path.exists(destination_dir) and not os.path.isdir(destination_dir):
        updates = [
            gr.update(value=f"Error: Cannot create folder '{new_folder_name}'. A file with that name already exists in {ASSETS_DIR}."),
            gr.update(root_dir=ASSETS_DIR, file_count="single", label=f"Assets Explorer (`{ASSETS_DIR}/`)"),
            gr.update(choices=get_asset_folder_choices(), interactive=True),
            gr.update(value=None),
            gr.update(visible=False)
        ]
        yield updates
        return

    try:
        os.makedirs(destination_dir, exist_ok=True)
        print(f"Ensured destination directory exists: {destination_dir}")
    except Exception as e:
        updates = [
            gr.update(value=f"Error creating destination folder '{new_folder_name}': {e}"),
            gr.update(root_dir=ASSETS_DIR, file_count="single", label=f"Assets Explorer (`{ASSETS_DIR}/`)"),
            gr.update(choices=get_asset_folder_choices(), interactive=True),
            gr.update(value=None),
            gr.update(visible=False)
        ]
        yield updates
        return

    upload_log = []
    success_count = 0
    # Consider making required_files more flexible or clearly communicated to the user
    required_files_to_copy = {'right.jpg', 'left.jpg', 'front.jpg', 'raw.mp4'} 
    copied_actual_files = set()

    for temp_item_path in temp_item_paths:
        if not os.path.exists(temp_item_path): # Check if temp file still exists
            print(f"Warning: Temporary file {temp_item_path} not found, skipping.")
            upload_log.append(f"Skipped non-existent temp file: {os.path.basename(temp_item_path)}")
            continue

        file_name = os.path.basename(temp_item_path)
        # We copy all uploaded files, then check if the required ones for ffmpeg are present
        final_destination_path = os.path.join(destination_dir, file_name)

        print(f"About to copy {temp_item_path} to {final_destination_path}")
        try:
            shutil.copy2(temp_item_path, final_destination_path)
            upload_log.append(f"Copied file: {file_name} -> {os.path.join(new_folder_name, file_name)}")
            copied_actual_files.add(file_name)
            if file_name in required_files_to_copy:
                 success_count +=1 # Counts towards the "required" files specifically
            print(f"Copied {file_name} to {final_destination_path}")

        except Exception as e:
            error_msg = f"Error copying '{file_name}': {e}"
            upload_log.append(error_msg)
            print(f"Error copying {temp_item_path} to {final_destination_path}: {e}")
            continue # Skip to next file if copy fails

    # FFmpeg processing for raw.mp4 if it was successfully copied
    if 'raw.mp4' in copied_actual_files:
        raw_mp4_path = os.path.join(destination_dir, 'raw.mp4')
        split_dir = os.path.join(destination_dir, 'split')
        try:
            os.makedirs(split_dir, exist_ok=True)
            print(f"Created split directory: {split_dir}")
        except Exception as e:
            upload_log.append(f"Error creating 'split' directory: {e}")
            print(f"Error creating split directory: {e}")
        else: # Only proceed if split_dir was created
            output_pattern = os.path.join(split_dir, 'frame_%04d.jpg')
            ffmpeg_cmd = [
                'ffmpeg',
                '-i', raw_mp4_path,
                '-vf', 'fps=20', # Consider making fps configurable
                '-q:v', '2',     # Consider making quality configurable
                output_pattern
            ]
            try:
                result = subprocess.run(
                    ffmpeg_cmd,
                    capture_output=True,
                    text=True,
                    check=True,
                    creationflags=subprocess.CREATE_NO_WINDOW if platform.system() == "Windows" else 0 # Hide console on Windows
                )
                upload_log.append(f"Split raw.mp4 into frames in '{os.path.join(new_folder_name, 'split')}'")
                print(f"ffmpeg output: {result.stdout}")
            except subprocess.CalledProcessError as e:
                error_msg = f"Error splitting raw.mp4 with ffmpeg: {e.stderr}"
                upload_log.append(error_msg)
                print(error_msg)
            except FileNotFoundError:
                error_msg = "Error: ffmpeg is not installed or not found in system PATH."
                upload_log.append(error_msg)
                print(error_msg)
    elif 'raw.mp4' in required_files_to_copy: # If it was required but not copied
        upload_log.append("raw.mp4 was not successfully copied, skipping frame splitting.")


    final_status_message = (f"Finished processing uploaded items into '{new_folder_name}'. "
                            f"{len(copied_actual_files)} file(s) copied. "
                            f"{success_count}/{len(required_files_to_copy)} required files for pipeline found.\n" +
                            "\n".join(upload_log))


    final_new_choices = get_asset_folder_choices()
    final_dropdown_value = new_folder_name if new_folder_name in final_new_choices else (final_new_choices[0] if final_new_choices else None)

    # Refresh assets_file_explorer by creating a new instance with updated root
    # This is a workaround if gr.update(root_dir=...) doesn't refresh as expected
    refreshed_explorer = gr.FileExplorer(
            root_dir=ASSETS_DIR,
            file_count="single",
            label=f"Assets Explorer (`{ASSETS_DIR}/`)",
            glob='**/*',
            ignore_glob='**/.*',
            value=os.path.join(destination_dir) if os.path.exists(destination_dir) and final_dropdown_value == new_folder_name else None # Select the new folder
        )

    updates = [
        gr.update(value=final_status_message),
        refreshed_explorer,
        gr.update(choices=final_new_choices, value=final_dropdown_value, interactive=bool(final_new_choices)),
        gr.update(value=None), # Clear upload_state
        gr.update(visible=False) # Hide modal
    ]
    yield updates


def cancel_upload_modal():
    """Cancels the upload process initiated by the modal."""
    print("Upload modal cancelled.")
    updates = [
        gr.update(value="Upload cancelled."),
        gr.update(value=None),
        gr.update(visible=False)
    ]
    return updates

def refresh_explorer_update(explorer_root):
    """Returns a Gradio update object for a FileExplorer."""
    print(f"Refreshing explorer for root: {explorer_root}")
    label = "Assets Explorer" if explorer_root == ASSETS_DIR else "Results Explorer"
    # It's better to return a new component instance for FileExplorer to reliably refresh
    return gr.FileExplorer(
        root_dir=explorer_root,
        file_count="single",
        label=f"{label} (`{explorer_root}/`)",
        glob='**/*',
        ignore_glob='**/.*',
    )

# --- Gradio Interface ---
with gr.Blocks(title="MH-Pipeline", css=css, theme=gr.themes.Citrus()) as demo:
    upload_state = gr.State(value=None) # Stores list of temp file paths from UploadButton
    
    with Modal(visible=False) as upload_folder_modal: # Correctly use Modal context manager
        with gr.Column(elem_classes=["modal-content"]):
            gr.Markdown("## Enter Folder Name for Uploaded Assets")
            new_folder_name_textbox = gr.Textbox(label="New Folder Name (will be created inside ./assets)", interactive=True, placeholder="e.g., MyNewAsset")
            with gr.Row():
                confirm_upload_button = gr.Button("Create Folder and Upload", variant="primary")
                cancel_modal_button = gr.Button("Cancel")

    with gr.Column():
        with gr.Row():
            with gr.Column(scale=1):
                gr.Markdown("## Assets")
                assets_file_explorer = gr.FileExplorer(
                    root_dir=ASSETS_DIR,
                    file_count="single",
                    label=f"Assets Explorer (`{ASSETS_DIR}/`)",
                    glob='**/*',
                    ignore_glob='**/.*',
                )
                gr.Markdown("### Upload to Assets Folder")
                upload_button = gr.UploadButton(
                    "Click or Drop Folder(s) / File(s) Here",
                    file_count="multiple", # Changed to multiple to better handle lists of files
                    # file_types=["image", "video", ".fbx", ".abc", ".obj", ".glb", ".gltf", ".stl", ".ply"] # Example file types
                )
                upload_status = gr.Textbox(label="Upload Status", interactive=False, lines=3, max_lines=10) # Increased max_lines

            with gr.Column(scale=3):
                gr.Markdown("## MH-Pipeline - Reconstruction Settings")
                with gr.Row():
                    with gr.Column(scale=1):
                        generation_mode = gr.Dropdown(
                            label="Mode", choices=["Prototyping", "Production"], value="Prototyping"
                        )
                        initial_asset_folder_choices = get_asset_folder_choices()
                        default_asset_folder = initial_asset_folder_choices[0] if initial_asset_folder_choices else None
                        asset_folder_selector = gr.Dropdown(
                            label="Asset Folder (Select a subfolder)",
                            choices=initial_asset_folder_choices,
                            value=default_asset_folder,
                            interactive=bool(initial_asset_folder_choices)
                        )
                        parts_to_generate = gr.Radio(
                            label="Parts",
                            choices=["Face & Hair", "Face Only", "Hair Only"],
                            value="Face & Hair"
                        )
                        generate_button = gr.Button("Generate Model", variant="primary")
                    with gr.Column(scale=2):
                        estimated_time_display = gr.Textbox(label="Estimated Time", interactive=False, value="Calculating...")
                        system_info_display = gr.Textbox(label="System Info", value=get_system_info(), interactive=False, lines=3)


                gr.Markdown("---")
                with gr.Row():
                    with gr.Column(scale=1, elem_classes=["preview-col"]):
                        gr.Markdown("### Asset Preview")
                        with gr.Group(elem_classes=["fixed-height-preview"]):
                            assets_image_preview = gr.Image(label="Image Preview", visible=False, show_label=False, elem_classes=["visualized-file"])
                            assets_video_preview = gr.Video(label="Video Preview", visible=False, show_label=False, elem_classes=["visualized-file"])
                            assets_model_preview = gr.Model3D(label="3D Model Preview", visible=False, show_label=False, clear_color=[0.0, 0.0, 0.0, 0.0], elem_classes=["visualized-file"])
                            assets_status_preview = gr.Markdown("", elem_classes=["status-message"])
                    with gr.Column(scale=1, elem_classes=["preview-col"]):
                        gr.Markdown("### Results Preview")
                        with gr.Group(elem_classes=["fixed-height-preview"]):
                            results_image_preview = gr.Image(label="Image Preview", visible=False, show_label=False, elem_classes=["visualized-file"])
                            results_video_preview = gr.Video(label="Video Preview", visible=False, show_label=False, elem_classes=["visualized-file"])
                            results_model_preview = gr.Model3D(label="3D Model Preview", visible=False, show_label=False, clear_color=[0.0, 0.0, 0.0, 0.0], elem_classes=["visualized-file"])
                            results_status_preview = gr.Markdown("", elem_classes=["status-message"])

            with gr.Column(scale=1):
                gr.Markdown("## Results")
                results_file_explorer = gr.FileExplorer(
                    root_dir=RESULTS_DIR,
                    file_count="single",
                    label=f"Results Explorer (`{RESULTS_DIR}/`)",
                    glob='**/*',
                    ignore_glob='**/.*',
                )
                refresh_results_btn = gr.Button("Refresh Results List")
                terminal_output = gr.Textbox(label="Generation Log", lines=10, interactive=False, autoscroll=True)


        # --- Event Handlers ---
        assets_file_explorer.change(
            visualize_file,
            inputs=[assets_file_explorer],
            outputs=[assets_image_preview, assets_video_preview, assets_model_preview, assets_status_preview]
        )
        
        results_file_explorer.change(
            visualize_file,
            inputs=[results_file_explorer],
            outputs=[results_image_preview, results_video_preview, results_model_preview, results_status_preview]
        )

        # When files are uploaded via the button, trigger the modal and store the temp file paths
        upload_button.upload(
            trigger_folder_upload_modal,
            inputs=[upload_button], # upload_button contains the list of temp file paths
            outputs=[upload_status, upload_state, upload_folder_modal, new_folder_name_textbox]
        )

        # When "Create Folder and Upload" in modal is clicked
        confirm_upload_button.click(
            copy_uploaded_items_to_subfolder,
            inputs=[upload_state, new_folder_name_textbox, asset_folder_selector],
            outputs=[upload_status, assets_file_explorer, asset_folder_selector, upload_state, upload_folder_modal]
        )

        # When "Cancel" in modal is clicked
        cancel_modal_button.click(
            cancel_upload_modal,
            inputs=[],
            outputs=[upload_status, upload_state, upload_folder_modal]
        ).then(
            lambda: "", outputs=[new_folder_name_textbox] # Clear the textbox on cancel
        )


        estimate_time_inputs = [generation_mode, asset_folder_selector, parts_to_generate]
        for comp in estimate_time_inputs:
            comp.change(estimate_generation_time, inputs=estimate_time_inputs, outputs=estimated_time_display)

        demo.load(estimate_generation_time, inputs=estimate_time_inputs, outputs=estimated_time_display)
        demo.load(lambda: get_asset_folder_choices(), inputs=None, outputs=asset_folder_selector) # Load initial choices

        generate_button.click(
            generate_model,
            inputs=[generation_mode, asset_folder_selector, parts_to_generate, estimated_time_display],
            outputs=[terminal_output]
        ).then( # Add a .then() to refresh the results explorer after generation
            lambda: refresh_explorer_update(RESULTS_DIR),
            inputs=[],
            outputs=[results_file_explorer]
        )

        refresh_results_btn.click(
            lambda: refresh_explorer_update(RESULTS_DIR),
            inputs=[],
            outputs=[results_file_explorer]
        )
        # Refresh asset folder choices when a new folder is created or on load
        confirm_upload_button.click(
            None, # No function, just trigger .then()
            inputs=None,
            outputs=None
        ).then(
            fn=lambda: gr.update(choices=get_asset_folder_choices()),
            inputs=None,
            outputs=asset_folder_selector
        )


# --- Launch the App ---
if __name__ == "__main__":
    # Ensure directories exist (already done at the top, but good for __main__)
    os.makedirs(ASSETS_DIR, exist_ok=True)
    os.makedirs(RESULTS_DIR, exist_ok=True)
    os.makedirs(GRADIO_TEMP_DIR, exist_ok=True)
    
    print(f"Assets directory: {os.path.abspath(ASSETS_DIR)}")
    print(f"Results directory: {os.path.abspath(RESULTS_DIR)}")
    print(f"Gradio Temp directory: {os.path.abspath(GRADIO_TEMP_DIR)}")
    
    demo.launch(share=True, allowed_paths=[ASSETS_DIR, RESULTS_DIR, GRADIO_TEMP_DIR], show_error=True, debug=True)