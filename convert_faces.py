import bpy
import os
import glob

# Take every front.ply file in ../results/*/hair3D/ and convert it to .glb file
## First get all directories in ../results
dirs = glob.glob('./results/*/')
print(dirs)
obj_files = dict()
for dir in dirs:
    obj_files[dir] = glob.glob(dir + '/NextFace/front.png/mesh0.obj')
print(obj_files)

# Clear existing scene data
def clear_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()

# Center the object at the origin
def center_object(obj):
    # Select the object
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    
    # Set origin to geometry center (fixes offset)
    bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY', center='BOUNDS')
    
    # Move object to world origin (0, 0, 0)
    obj.location = (0, 0, 0)

# Normalize object scale to a standard size
def normalize_scale(obj, target_size=1.0):
    # Select the object
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    
    # Calculate the maximum dimension of the object
    dimensions = obj.dimensions
    max_dim = max(dimensions.x, dimensions.y, dimensions.z)
    
    if max_dim > 0:  # Prevent division by zero
        scale_factor = target_size / max_dim
        obj.scale = (scale_factor, scale_factor, scale_factor)
        
        # Apply the scale transformation
        bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    
    return max_dim, scale_factor

def main(obj_input_path, glb_output_path):
    # Clear the scene
    clear_scene()

    # Import the OBJ file with materials and textures
    bpy.ops.import_scene.obj(filepath=obj_input_path, use_image_search=True)
    
    # Get all imported mesh objects
    imported_objects = [obj for obj in bpy.context.scene.objects if obj.type == 'MESH']
    if not imported_objects:
        print("No mesh objects imported!")
        return
    
    # Process each imported mesh object
    for obj in imported_objects:
        # Center the object
        center_object(obj)
        print(f"Centered object: {obj.name}")
        
        # Normalize scale (target_size=1.0 means max dimension will be 1 unit)
        original_size, scale_factor = normalize_scale(obj)
        print(f"Normalized {obj.name}: Original max dimension = {original_size:.3f}, Scale factor = {scale_factor:.3f}")

    # Verify materials
    for obj in imported_objects:
        if obj.data.materials:
            print(f"Object {obj.name} has {len(obj.data.materials)} materials.")
        else:
            print(f"Warning: Object {obj.name} has no materials.")
    
    # Pack all external textures to include them in the .glb file
    bpy.ops.file.pack_all()

    # Export to GLB with materials and textures
    bpy.ops.export_scene.gltf(
        filepath=glb_output_path,
        export_format='GLB',  # Binary format, all textures packed
        export_image_format='AUTO',  # Use the same format as the original image
        use_selection=False,  # Export all objects in scene
        export_apply=True,  # Apply all transforms
        export_texcoords=True,
        export_normals=True,
        export_materials='EXPORT',  # Ensure materials are exported
        export_colors=True,  # Include vertex colors if present
        export_lights=False,
        export_cameras=False,  # Skip cameras
    )
    print(f"Exported to {glb_output_path}")

if __name__ == "__main__":
    for dir, files in obj_files.items():
        for obj_file in files:
            glb_file = os.path.join(dir, 'NextFace.glb')
            main(obj_file, glb_file)
            print(f"Conversion complete: {obj_file} -> {glb_file}")
