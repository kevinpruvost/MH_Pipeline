import gradio as gr
import os
import time
import platform
import psutil # Keep if needed for other functions, removed from generate_model logic
import pandas as pd # Keep if needed for other functions, removed from generate_model logic
import math # Keep if needed for other functions, removed from generate_model logic
import shutil
import re # Keep if needed for other functions, removed from generate_model logic
import subprocess
import GPUtil # Keep if needed for other functions, removed from generate_model logic
from gradio_modal import Modal # Keep if needed elsewhere or defined

import shlex

# Define base directories
ASSETS_DIR = "./assets"
RESULTS_DIR = "./results"
SCRIPTS_DIR = "./"

# Ensure results and scripts directories exist
os.makedirs(RESULTS_DIR, exist_ok=True)
os.makedirs(SCRIPTS_DIR, exist_ok=True)

# --- Script Configuration ---
# This dictionary acts as the "table of scripts".
# Key: (generation_mode, parts_to_generate)
# Value: Path to the script to execute
SCRIPT_MAP = {
    ("Prototyping", "Face Only"): [os.path.join(SCRIPTS_DIR, "3ddfav3.sh")],
    ("Prototyping", "Hair Only"): [os.path.join(SCRIPTS_DIR, "hairstep.sh")],
    ("Production", "Face Only"): [os.path.join(SCRIPTS_DIR, "nextFace.sh")],
    ("Production", "Hair Only"): [os.path.join(SCRIPTS_DIR, "gaussianHaircut.sh")],
}
SCRIPT_MAP[("Prototyping", "Face & Hair")] = SCRIPT_MAP[("Prototyping", "Face Only")] + SCRIPT_MAP[("Prototyping", "Hair Only")] 
SCRIPT_MAP[("Production", "Face & Hair")] = SCRIPT_MAP[("Production", "Face Only")] + SCRIPT_MAP[("Production", "Hair Only")]
# --------------------------

def launch_script(script_path, asset_folder_path, parts_to_generate, generation_mode):
    if not os.path.exists(script_path):
        error_msg = f"Error: Script file '{script_path}' not found."
        print(error_msg)
        yield error_msg
        # Log error
        with open(log_filepath, "w") as f:
             f.write(f"--- Generation Dispatch Log ---\n")
             f.write(f"Timestamp: {timestamp}\n")
             f.write(f"Error: Script file '{script_path}' not found.\n")
             f.write(f"\n--- End of Log ---\n")
        return

    # Ensure script is executable (mostly relevant for Unix-like systems)
    if platform.system() != "Windows" and not os.access(script_path, os.X_OK):
        try:
            os.chmod(script_path, os.stat(script_path).st_mode | 0o111)
            print(f"Made script '{script_path}' executable.")
            yield f"Made script '{script_path}' executable."
        except Exception as e:
            error_msg = f"Error: Script file '{script_path}' is not executable and failed to make it executable: {e}"
            print(error_msg)
            yield error_msg
            # Log error
            with open(log_filepath, "w") as f:
                 f.write(f"--- Generation Dispatch Log ---\n")
                 f.write(f"Timestamp: {timestamp}\n")
                 f.write(f"Error: Failed to make script executable '{script_path}': {e}\n")
                 f.write(f"\n--- End of Log ---\n")
            return


    # --- Construct Commands ---
    tmux_session_name = "MH-Pipeline"
    # Create a unique window name for each run
    tmux_window_name = f"gen_{timestamp}_{parts_to_generate.replace(' & ', '_').replace(' ', '_').lower()}"
    # Limit window name length and remove potentially problematic characters
    tmux_window_name = re.sub(r'[^a-zA-Z0-9_-]', '', tmux_window_name)[:50]


    # Construct the command string to run inside tmux
    # Use shlex.quote to handle spaces and special characters in paths
    quoted_script_path = shlex.quote(script_path)
    quoted_asset_folder_path = shlex.quote(full_asset_folder_path)
    # Append 'bash -l' to keep the pane open after the script finishes
    command_to_run_in_tmux = f"{quoted_script_path} {quoted_asset_folder_path}; bash -l"

    yield f"Checking for existing tmux session '{tmux_session_name}'..."

    # --- Tmux Session Check and Command Dispatch ---
    try:
        # Check if the tmux session exists
        check_session_cmd = ["tmux", "has-session", "-t", tmux_session_name]
        print(f"Running: {' '.join(check_session_cmd)}")
        # Redirect output to null to keep console clean
        session_check_result = subprocess.run(
            check_session_cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False # Do not raise an exception if the session doesn't exist
        )
        session_exists = session_check_result.returncode == 0

        if session_exists:
            yield f"Tmux session '{tmux_session_name}' found. Creating new window '{tmux_window_name}' and sending command..."
            # Create a new window in the existing session and run the command
            tmux_command = [
                "tmux", "new-window",
                "-t", f"{tmux_session_name}:", # Target the session, let tmux assign window index
                "-n", tmux_window_name,       # Name the new window
                command_to_run_in_tmux        # The command string to execute
            ]
            print(f"Dispatching command to new tmux window: {' '.join(tmux_command)}")
            yield "Dispatching command to new tmux window..."

        else:
            yield f"Tmux session '{tmux_session_name}' not found. Creating new detached session and initial window '{tmux_window_name}'..."
            # Create a new detached session and run the command in its first window
            tmux_command = [
                "tmux", "new-session",
                "-d",                  # Detached session
                "-s", tmux_session_name, # Session name
                "-n", tmux_window_name,  # First window name
                command_to_run_in_tmux   # The command string to execute
            ]
            print(f"Dispatching command to new tmux session: {' '.join(tmux_command)}")
            yield "Dispatching command to new tmux session and window..."

        # Execute the tmux command (either new-window or new-session)
        # We run this detached, so we won't wait for the script to finish
        tmux_process = subprocess.run(
            tmux_command,
            capture_output=True, # Capture output of the tmux command itself (for debugging tmux issues)
            text=True,
            check=False, # Don't raise an exception for non-zero exit codes from tmux itself
            cwd="." # Run from the current directory
        )

        # Log the details of the tmux command execution
        with open(log_filepath, "w") as f:
            f.write(f"--- Generation Dispatch Log (tmux) ---\n")
            f.write(f"Timestamp: {timestamp}\n")
            f.write(f"Mode: {generation_mode}\n")
            f.write(f"Parts: {parts_to_generate}\n")
            f.write(f"Asset Folder: {full_asset_folder_path}\n")
            f.write(f"Tmux Session Name: {tmux_session_name}\n")
            f.write(f"Tmux Window Name: {tmux_window_name}\n")
            f.write(f"Command dispatched to tmux: {command_to_run_in_tmux}\n")
            f.write(f"Tmux Execution Command: {' '.join(tmux_command)}\n")
            f.write(f"Tmux Command Exit Code: {tmux_process.returncode}\n")
            f.write(f"\n--- Tmux Command STDOUT ---\n")
            f.write(tmux_process.stdout)
            f.write(f"\n--- Tmux Command STDERR ---\n")
            f.write(tmux_process.stderr)
            f.write(f"\n--- End of Log ---\n")

        if tmux_process.returncode == 0:
            final_status = (
                f"Successfully dispatched command to tmux session '{tmux_session_name}', window '{tmux_window_name}'.\n"
                f"Check the tmux session (e.g., `tmux attach -t {tmux_session_name}` and navigate to window '{tmux_window_name}') "
                f"for script progress and output.\nLog saved to '{log_filepath}'."
            )
            print("Tmux command executed successfully.")
            if tmux_process.stdout: print("Tmux STDOUT:", tmux_process.stdout)
            if tmux_process.stderr: print("Tmux STDERR:", tmux_process.stderr)
            yield final_status
        else:
            error_msg = (
                f"Failed to dispatch command to tmux. Tmux command exited with code {tmux_process.returncode}.\n"
                f"Log saved to '{log_filepath}'.\n"
                f"Tmux STDOUT:\n{tmux_process.stdout}\n"
                f"Tmux STDERR:\n{tmux_process.stderr}"
            )
            print(f"Tmux command failed with exit code {tmux_process.returncode}.")
            if tmux_process.stdout: print("Tmux STDOUT:", tmux_process.stdout)
            if tmux_process.stderr: print("Tmux STDERR:", tmux_process.stderr)
            yield error_msg

    except FileNotFoundError:
        error_msg = "Error: 'tmux' command not found. Make sure tmux is installed and in your PATH."
        print(error_msg)
        yield error_msg
        # Log this specific error
        with open(log_filepath, "w") as f:
             f.write(f"--- Generation Dispatch Log ---\n")
             f.write(f"Timestamp: {timestamp}\n")
             f.write(f"Error: tmux command not found.\n")
             f.write(f"\n--- End of Log ---\n")

    except Exception as e:
        error_msg = f"An unexpected error occurred while running the tmux command: {e}"
        print(error_msg)
        yield error_msg
        # Log this error
        with open(log_filepath, "w") as f:
             f.write(f"--- Generation Dispatch Log ---\n")
             f.write(f"Timestamp: {timestamp}\n")
             f.write(f"Error during tmux command execution: {e}\n")
             f.write(f"\n--- End of Log ---\n")

def generate_model(generation_mode: str, asset_folder_name: str, parts_to_generate: str, estimated_time):
    """
    Selects a script based on generation mode and parts, and launches it
    within a tmux session named 'MH-Pipeline'.
    """
    global full_asset_folder_path, script_key
    full_asset_folder_path = os.path.join(ASSETS_DIR, asset_folder_name)
    script_key = (generation_mode, parts_to_generate)

    global log_filepath, timestamp, log_filename
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    log_filename = f"generation_dispatch_log_{timestamp}.txt"
    log_filepath = os.path.join(RESULTS_DIR, log_filename)

    print(f"Attempting to dispatch generation task to tmux: Mode='{generation_mode}', Parts='{parts_to_generate}', Asset Folder='{full_asset_folder_path}'")
    yield f"Preparing to dispatch task: Mode='{generation_mode}', Parts='{parts_to_generate}'..."

    # --- Input Validation ---
    if not asset_folder_name:
         error_msg = "Error: No asset folder name provided."
         print(error_msg)
         yield error_msg
         # Log error
         with open(log_filepath, "w") as f:
             f.write(f"--- Generation Dispatch Log ---\n")
             f.write(f"Timestamp: {timestamp}\n")
             f.write(f"Error: No asset folder name provided.\n")
             f.write(f"\n--- End of Log ---\n")
         return

    if not os.path.isdir(full_asset_folder_path):
        error_msg = f"Error: Asset folder '{full_asset_folder_path}' not found or is not a directory."
        print(error_msg)
        yield error_msg
        # Log error
        with open(log_filepath, "w") as f:
             f.write(f"--- Generation Dispatch Log ---\n")
             f.write(f"Timestamp: {timestamp}\n")
             f.write(f"Error: Asset folder '{full_asset_folder_path}' not found.\n")
             f.write(f"\n--- End of Log ---\n")
        return

    if script_key not in SCRIPT_MAP:
        error_msg = f"Error: No script defined for Mode='{generation_mode}' and Parts='{parts_to_generate}'."
        print(error_msg)
        yield error_msg
        # Log error
        with open(log_filepath, "w") as f:
             f.write(f"--- Generation Dispatch Log ---\n")
             f.write(f"Timestamp: {timestamp}\n")
             f.write(f"Error: No script mapping for {script_key}.\n")
             f.write(f"\n--- End of Log ---\n")
        return


    script_paths = SCRIPT_MAP[script_key]

    for script_path in script_paths:
        launch_status = launch_script(script_path, full_asset_folder_path, parts_to_generate, generation_mode)
        for status in launch_status:
            yield status