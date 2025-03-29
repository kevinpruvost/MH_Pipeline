import bpy
import glob

# Take every front.ply file in ../results/*/hair3D/ and convert it to .abc file
dirs = glob.glob('../results/*/')
print(dirs)
ply_files = dict()
for dir in dirs:
    ply_files[dir] = glob.glob(dir + '/HairStep/hair3D/front.ply')
print(ply_files)

# Clear existing scene data
def clear_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()

# Center the object at the origin
def center_object(obj):
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY', center='BOUNDS')  # Set origin to geometry center
    obj.location = (0, 0, 0)  # Move to world origin

# Normalize object scale to a standard size
def normalize_scale(obj, target_size=1.0):
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    dimensions = obj.dimensions
    max_dim = max(dimensions.x, dimensions.y, dimensions.z)
    if max_dim > 0:  # Prevent division by zero
        scale_factor = target_size / max_dim
        obj.scale = (scale_factor, scale_factor, scale_factor)
        bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)  # Apply scale
    return max_dim, scale_factor

# Main conversion function
def convert_ply_to_abc(ply_input_path, abc_output_path):
    # Clear the scene
    clear_scene()

    # Import the PLY file
    bpy.ops.import_mesh.ply(filepath=ply_input_path)
    
    # Get the imported object
    if not bpy.context.selected_objects:
        print(f"No objects imported from {ply_input_path}!")
        return
    obj = bpy.context.selected_objects[0]
    bpy.context.view_layer.objects.active = obj

    # Ensure we're in Object mode
    bpy.ops.object.mode_set(mode='OBJECT')

    # Center the object
    center_object(obj)
    print(f"Centered object: {obj.name}")

    # Normalize scale (target_size=1.0 means max dimension will be 1 unit)
    original_size, scale_factor = normalize_scale(obj)
    print(f"Normalized {obj.name}: Original max dimension = {original_size:.3f}, Scale factor = {scale_factor:.3f}")

    # Convert mesh to curves (assuming this is for hair strands)
    bpy.ops.object.convert(target='CURVE')

    # Export to Alembic
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.ops.wm.alembic_export(
        filepath=abc_output_path,
        selected=True,
        visible_objects_only=False,
        export_hair=True,  # Ensure curves/hair are exported
    )
    print(f"Conversion complete: {ply_input_path} -> {abc_output_path}")

# Execute the conversion for all files
if __name__ == "__main__":
    for dir, files in ply_files.items():
        for ply_file in files:
            abc_file = dir + 'HairNet_Hair.abc'
            convert_ply_to_abc(ply_file, abc_file)