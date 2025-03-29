export CAMERA="PINHOLE"
export EXP_NAME_1="stage1"
export EXP_NAME_2="stage2"
export EXP_NAME_3="stage3"

eval "$(conda shell.bash hook)"

# Get GPU from command line argument
if [ $# -ne 2 ]; then
    echo "Error: Please provide GPU as an argument"
    echo "Usage: $0 <DATA_PATH2> <GPU>"
    exit 1
fi

export GPU="$2"
DATA_PATH2="$1"

#
# Ensure that the following environment variables are accessible to the script:
# PROJECT_DIR and DATA_PATH 
#
export PROJECT_DIR=$PWD
export BLENDER_DIR=$PWD/blender-3.6.13-linux-x64/blender
export DATA_PATH=$PWD/$DATA_PATH2

echo $DATA_PATH

export EXP_PATH_1=$DATA_PATH/3d_gaussian_splatting/$EXP_NAME_1

# Need to use this to activate conda environments

#################
# PREPROCESSING #
#################

# Arrange raw images into a 3D Gaussian Splatting format
if [[ ! -d "$DATA_PATH/images" && ! -d "$DATA_PATH/images_2" && ! -d "$DATA_PATH/masks" ]]; then
    echo "Arranging raw images..."
    conda deactivate && conda activate gaussian_splatting_hair
    cd $PROJECT_DIR/src/preprocessing
    mkdir -p ~/.cache/torch/hub/checkpoints/
    if [ ! -f ~/.cache/torch/hub/checkpoints/resnet50-19c8e357.pth ]; then
        wget https://download.pytorch.org/models/resnet50-19c8e357.pth -O ~/.cache/torch/hub/checkpoints/resnet50-19c8e357.pth
    fi
    CUDA_VISIBLE_DEVICES="$GPU" python preprocess_raw_images.py \
        --data_path $DATA_PATH
else
    echo "Skipping preprocess_raw_images.py: $DATA_PATH/images already exists"
fi

# Run COLMAP reconstruction and undistort the images and cameras
if [ ! -d "$DATA_PATH/sparse/0" ]; then
    echo "Running COLMAP reconstruction..."
    conda deactivate && conda activate gaussian_splatting_hair
    cd $PROJECT_DIR/src
    CUDA_VISIBLE_DEVICES="$GPU" python convert.py -s $DATA_PATH \
        --camera $CAMERA --max_size 1024
else
    echo "Skipping convert.py: $DATA_PATH/sparse/0 already exists"
fi

# Run Matte-Anything
if [[ ! -d "$DATA_PATH/masks" && ! -d "$DATA_PATH/masks_2" ]]; then
    echo "Running Matte-Anything..."
    conda deactivate && conda activate matte_anything
    cd $PROJECT_DIR/ext/Matte-Anything/GroundingDINO && pip install -e .
    cd $PROJECT_DIR/src/preprocessing
    CUDA_VISIBLE_DEVICES="$GPU" python calc_masks.py \
        --data_path $DATA_PATH --image_format png --max_size 2048

    # Filter images using their IQA scores
    echo "Filtering images by IQA scores..."
    conda deactivate && conda activate gaussian_splatting_hair
    cd $PROJECT_DIR/src/preprocessing
    CUDA_VISIBLE_DEVICES="$GPU" python filter_extra_images.py \
        --data_path $DATA_PATH --max_imgs 128
else
    echo "Skipping calc_masks.py: $DATA_PATH/masks already exists"
fi

# Resize images
if [ ! -d "$DATA_PATH/images_2" ]; then
    echo "Resizing images..."
    conda deactivate && conda activate gaussian_splatting_hair
    cd $PROJECT_DIR/src/preprocessing
    CUDA_VISIBLE_DEVICES="$GPU" python resize_images.py --data_path $DATA_PATH
else
    echo "Skipping resize_images.py: $DATA_PATH/images_2 already exists"
fi

# Calculate orientation maps
if [ ! -d "$DATA_PATH/orientations_2" ]; then
    echo "Calculating orientation maps..."
    conda deactivate && conda activate gaussian_splatting_hair
    cd $PROJECT_DIR/src/preprocessing
    CUDA_VISIBLE_DEVICES="$GPU" python calc_orientation_maps.py \
        --img_path $DATA_PATH/images_2 \
        --mask_path $DATA_PATH/masks_2/hair \
        --orient_dir $DATA_PATH/orientations_2/angles \
        --conf_dir $DATA_PATH/orientations_2/vars \
        --filtered_img_dir $DATA_PATH/orientations_2/filtered_imgs \
        --vis_img_dir $DATA_PATH/orientations_2/vis_imgs
else
    echo "Skipping calc_orientation_maps.py: $DATA_PATH/orientations_2 already exists"
fi

# Run OpenPose
if [ ! -d "$DATA_PATH/openpose" ]; then
    echo "Running OpenPose..."
    conda deactivate && cd $PROJECT_DIR/ext/openpose
    mkdir -p $DATA_PATH/openpose
    conda activate openpose
    CUDA_VISIBLE_DEVICES="$GPU" ./build/examples/openpose/openpose.bin \
        --image_dir $DATA_PATH/images_4 \
        --scale_number 4 --scale_gap 0.25 --face --hand --display 0 \
        --write_json $DATA_PATH/openpose/json \
        --write_images $DATA_PATH/openpose/images --write_images_format jpg
else
    echo "Skipping OpenPose: $DATA_PATH/openpose already exists"
fi

# Run Face-Alignment
conda deactivate && conda activate gaussian_splatting_hair
cd $PROJECT_DIR/src/preprocessing

# Ensure all required weights are in the cache
if [ ! -f ~/.cache/torch/hub/checkpoints/2DFAN4-cd938726ad.zip ]; then
    wget https://www.adrianbulat.com/downloads/python-fan/2DFAN4-cd938726ad.zip -O ~/.cache/torch/hub/checkpoints/2DFAN4-cd938726ad.zip
    unzip ~/.cache/torch/hub/checkpoints/2DFAN4-cd938726ad.zip -d ~/.cache/torch/hub/checkpoints/
fi
if [ ! -f ~/.cache/torch/hub/checkpoints/depth-6c4283c0e0.zip ]; then
    wget https://www.adrianbulat.com/downloads/python-fan/depth-6c4283c0e0.zip -O ~/.cache/torch/hub/checkpoints/depth-6c4283c0e0.zip
    unzip ~/.cache/torch/hub/checkpoints/depth-6c4283c0e0.zip -d ~/.cache/torch/hub/checkpoints/
fi
if [ ! -f ~/.cache/torch/hub/checkpoints/3DFAN4-4a694010b9.zip ]; then
    wget https://www.adrianbulat.com/downloads/python-fan/3DFAN4-4a694010b9.zip -O ~/.cache/torch/hub/checkpoints/3DFAN4-4a694010b9.zip
    unzip ~/.cache/torch/hub/checkpoints/3DFAN4-4a694010b9.zip -d ~/.cache/torch/hub/checkpoints/
fi
if [ ! -f ~/.cache/torch/hub/checkpoints/resnet50-0676ba61.pth ]; then
    wget https://download.pytorch.org/models/resnet50-0676ba61.pth -O ~/.cache/torch/hub/checkpoints/resnet50-0676ba61.pth
fi
if [ ! -f ~/.cache/torch/hub/checkpoints/3DFAN4-7835d9f11d.pth.tar ]; then
    wget https://www.adrianbulat.com/downloads/python-fan/3DFAN4-7835d9f11d.pth.tar -O ~/.cache/torch/hub/checkpoints/3DFAN4-7835d9f11d.pth.tar
fi
if [ ! -f ~/.cache/torch/hub/checkpoints/depth-2a464da4ea.pth.tar ]; then
    wget https://www.adrianbulat.com/downloads/python-fan/depth-2a464da4ea.pth.tar -O ~/.cache/torch/hub/checkpoints/depth-2a464da4ea.pth.tar
fi
if [ ! -f ~/.cache/torch/hub/checkpoints/s3fd-619a316812.pth ]; then
    wget https://www.adrianbulat.com/downloads/python-fan/s3fd-619a316812.pth -O ~/.cache/torch/hub/checkpoints/s3fd-619a316812.pth
fi

if [[ ! -f "$DATA_PATH/face_alignment/lmks_2d.pkl" && ! -f "$DATA_PATH/face_alignment/lmks_3d.pkl" ]]; then
    echo "Running Face-Alignment..."
    CUDA_VISIBLE_DEVICES="$GPU" python calc_face_alignment.py \
        --data_path $DATA_PATH --image_dir "images_4"
else
    echo "Skipping calc_face_alignment.py: $DATA_PATH/face_alignment/lmks_2d.pkl already exists"
fi

# Run PIXIE
if [ ! -d "$DATA_PATH/pixie" ]; then
    echo "Running PIXIE..."
    conda deactivate && conda activate pixie-env
    cd $PROJECT_DIR/ext/PIXIE
    CUDA_VISIBLE_DEVICES="$GPU" python demos/demo_fit_face.py \
        -i $DATA_PATH/images_4 -s $DATA_PATH/pixie \
        --saveParam True --lightTex False --useTex False \
        --rasterizer_type pytorch3d
else
    echo "Skipping demo_fit_face.py: $DATA_PATH/pixie already exists"
fi

# Merge all PIXIE predictions in a single file
if [ ! -f "$DATA_PATH/initialization_pixie" ]; then
    echo "Merging PIXIE predictions..."
    conda deactivate && conda activate gaussian_splatting_hair
    cd $PROJECT_DIR/src/preprocessing
    CUDA_VISIBLE_DEVICES="$GPU" python merge_smplx_predictions.py \
        --data_path $DATA_PATH
else
    echo "Skipping merge_smplx_predictions.py: $DATA_PATH/initialization_pixie already exists"
fi

# Convert COLMAP cameras to txt
if [ ! -d "$DATA_PATH/sparse_txt" ]; then
    echo "Converting COLMAP cameras to txt..."
    conda deactivate && conda activate gaussian_splatting_hair
    mkdir -p $DATA_PATH/sparse_txt
    CUDA_VISIBLE_DEVICES="$GPU" colmap model_converter \
        --input_path $DATA_PATH/sparse/0 \
        --output_path $DATA_PATH/sparse_txt --output_type TXT
else
    echo "Skipping model_converter: $DATA_PATH/sparse_txt already exists"
fi

# Convert COLMAP cameras to H3DS format
if [[ ! -f "$DATA_PATH/cameras.npz" && ! -f "$DATA_PATH/point_cloud.ply" ]]; then
    echo "Converting COLMAP cameras to H3DS format..."
    conda deactivate && conda activate gaussian_splatting_hair
    cd $PROJECT_DIR/src/preprocessing
    CUDA_VISIBLE_DEVICES="$GPU" python colmap_parsing.py \
        --path_to_scene $DATA_PATH
else
    echo "Skipping colmap_parsing.py: $DATA_PATH/cameras.npz already exists"
fi

# Remove raw files to preserve disk space
# rm -rf $DATA_PATH/input $DATA_PATH/images $DATA_PATH/masks $DATA_PATH/iqa*

##################
# RECONSTRUCTION #
##################

# Takes 45 minutes
# Run 3D Gaussian Splatting reconstruction
if [ ! -f "$EXP_PATH_1/point_cloud/iteration_30000/point_cloud.ply" ]; then
    echo "Running 3D Gaussian Splatting reconstruction..."
    conda activate gaussian_splatting_hair && cd $PROJECT_DIR/src
    CUDA_VISIBLE_DEVICES="$GPU" python train_gaussians.py \
        -s $DATA_PATH -m "$EXP_PATH_1" -r 1 --port "888$GPU" \
        --trainable_cameras --trainable_intrinsics --use_barf \
        --lambda_dorient 0.1
else
    echo "Skipping train_gaussians.py: $EXP_PATH_1/point_cloud/iteration_30000/point_cloud.ply already exists"
fi

# Run FLAME mesh fitting (3 stages)
if [ ! -d "$DATA_PATH/flame_fitting/$EXP_NAME_1/stage_3" ]; then
    echo "Running FLAME mesh fitting..."
    conda activate gaussian_splatting_hair
    cd $PROJECT_DIR/ext/NeuralHaircut/src/multiview_optimization

    if [ ! -d "$DATA_PATH/flame_fitting/$EXP_NAME_1/stage_1" ]; then
        CUDA_VISIBLE_DEVICES="$GPU" python fit.py --conf confs/train_person_1.conf \
            --batch_size 1 --train_rotation True --fixed_images True \
            --save_path $DATA_PATH/flame_fitting/$EXP_NAME_1/stage_1 \
            --data_path $DATA_PATH \
            --fitted_camera_path $EXP_PATH_1/cameras/30000_matrices.pkl
    else
        echo "Skipping stage 1 of FLAME fitting: $DATA_PATH/flame_fitting/$EXP_NAME_1/stage_1 already exists"
    fi

    if [ ! -d "$DATA_PATH/flame_fitting/$EXP_NAME_1/stage_2" ]; then
        CUDA_VISIBLE_DEVICES="$GPU" python fit.py --conf confs/train_person_1.conf \
            --batch_size 4 --train_rotation True --fixed_images True \
            --save_path $DATA_PATH/flame_fitting/$EXP_NAME_1/stage_2 \
            --checkpoint_path $DATA_PATH/flame_fitting/$EXP_NAME_1/stage_1/opt_params_final \
            --data_path $DATA_PATH \
            --fitted_camera_path $EXP_PATH_1/cameras/30000_matrices.pkl
    else
        echo "Skipping stage 2 of FLAME fitting: $DATA_PATH/flame_fitting/$EXP_NAME_1/stage_2 already exists"
    fi

    if [ ! -d "$DATA_PATH/flame_fitting/$EXP_NAME_1/stage_3" ]; then
        CUDA_VISIBLE_DEVICES="$GPU" python fit.py --conf confs/train_person_1_.conf \
            --batch_size 32 --train_rotation True --train_shape True \
            --save_path $DATA_PATH/flame_fitting/$EXP_NAME_1/stage_3 \
            --checkpoint_path $DATA_PATH/flame_fitting/$EXP_NAME_1/stage_2/opt_params_final \
            --data_path $DATA_PATH \
            --fitted_camera_path $EXP_PATH_1/cameras/30000_matrices.pkl
    else
        echo "Skipping stage 3 of FLAME fitting: $DATA_PATH/flame_fitting/$EXP_NAME_1/stage_3 already exists"
    fi
else
    echo "Skipping FLAME fitting: $DATA_PATH/flame_fitting/$EXP_NAME_1/stage_3 already exists"
fi

# Crop the reconstructed scene
if [ ! -f "$EXP_PATH_1/point_cloud/iteration_30000/point_cloud_cropped.ply" ]; then
    echo "Cropping reconstructed scene..."
    conda activate gaussian_splatting_hair && cd $PROJECT_DIR/src/preprocessing
    CUDA_VISIBLE_DEVICES="$GPU" python scale_scene_into_sphere.py \
        --path_to_data $DATA_PATH \
        -m "$EXP_PATH_1" --iter 30000
else
    echo "Skipping scale_scene_into_sphere.py: $EXP_PATH_1/point_cloud/iteration_30000/point_cloud_cropped.ply already exists"
fi

# Remove hair Gaussians that intersect with the FLAME head mesh
if [ ! -f "$EXP_PATH_1/point_cloud_filtered/iteration_30000/point_cloud.ply" ]; then
    echo "Filtering FLAME intersections..."
    conda activate gaussian_splatting_hair && cd $PROJECT_DIR/src/preprocessing
    CUDA_VISIBLE_DEVICES="$GPU" python filter_flame_intersections.py \
        --flame_mesh_dir $DATA_PATH/flame_fitting/$EXP_NAME_1 \
        -m "$EXP_PATH_1" --iter 30000 \
        --project_dir $PROJECT_DIR/ext/NeuralHaircut
else
    echo "Skipping filter_flame_intersections.py: $EXP_PATH_1/point_cloud_filtered/iteration_30000/point_cloud.ply already exists"
fi

# Run rendering for training views
if [ ! -d "$EXP_PATH_1/train_cropped" ]; then
    echo "Rendering training views..."
    conda activate gaussian_splatting_hair && cd $PROJECT_DIR/src
    CUDA_VISIBLE_DEVICES="$GPU" python render_gaussians.py \
        -s $DATA_PATH -m "$EXP_PATH_1" \
        --skip_test --scene_suffix "_cropped" --iteration 30000 \
        --trainable_cameras --trainable_intrinsics --use_barf
else
    echo "Skipping render_gaussians.py: $EXP_PATH_1/train_cropped already exists"
fi

# Get FLAME mesh scalp maps
if [ ! -d "$DATA_PATH/flame_fitting/$EXP_NAME_1/scalp_data" ]; then
    echo "Extracting FLAME mesh scalp maps..."
    conda activate gaussian_splatting_hair && cd $PROJECT_DIR/src/preprocessing
    CUDA_VISIBLE_DEVICES="$GPU" python extract_non_visible_head_scalp.py \
        --project_dir $PROJECT_DIR/ext/NeuralHaircut --data_dir $DATA_PATH \
        --flame_mesh_dir $DATA_PATH/flame_fitting/$EXP_NAME_1 \
        --cams_path $EXP_PATH_1/cameras/30000_matrices.pkl \
        -m "$EXP_PATH_1"
else
    echo "Skipping extract_non_visible_head_scalp.py: $DATA_PATH/flame_fitting/$EXP_NAME_1/scalp_data already exists"
fi

# Run latent hair strands reconstruction
if [ ! -f "$DATA_PATH/strands_reconstruction/$EXP_NAME_2/checkpoints/20000.pth" ]; then
    echo "Running latent hair strands reconstruction..."
    conda activate gaussian_splatting_hair && cd $PROJECT_DIR/src
    CUDA_VISIBLE_DEVICES="$GPU" python train_latent_strands.py \
        -s $DATA_PATH -m "$EXP_PATH_1" -r 1 \
        --model_path_hair "$DATA_PATH/strands_reconstruction/$EXP_NAME_2" \
        --flame_mesh_dir "$DATA_PATH/flame_fitting/$EXP_NAME_1" \
        --pointcloud_path_head "$EXP_PATH_1/point_cloud_filtered/iteration_30000/raw_point_cloud.ply" \
        --hair_conf_path "$PROJECT_DIR/src/arguments/hair_strands_textured.yaml" \
        --lambda_dmask 0.1 --lambda_dorient 0.1 --lambda_dsds 0.01 \
        --load_synthetic_rgba --load_synthetic_geom --binarize_masks --iteration_data 30000 \
        --trainable_cameras --trainable_intrinsics --use_barf \
        --iterations 20000 --port "800$GPU"
else
    echo "Skipping train_latent_strands.py: $DATA_PATH/strands_reconstruction/$EXP_NAME_2/checkpoints/20000.pth already exists"
fi

# Run hair strands reconstruction
# 2:30 hours
if [ ! -f "$DATA_PATH/curves_reconstruction/$EXP_NAME_3/checkpoints/10000.pth" ]; then
    echo "Running hair strands reconstruction..."
    conda activate gaussian_splatting_hair && cd $PROJECT_DIR/src
    CUDA_VISIBLE_DEVICES="$GPU" python train_strands.py \
        -s $DATA_PATH -m "$EXP_PATH_1" -r 1 \
        --model_path_curves "$DATA_PATH/curves_reconstruction/$EXP_NAME_3" \
        --flame_mesh_dir "$DATA_PATH/flame_fitting/$EXP_NAME_1" \
        --pointcloud_path_head "$EXP_PATH_1/point_cloud_filtered/iteration_30000/raw_point_cloud.ply" \
        --start_checkpoint_hair "$DATA_PATH/strands_reconstruction/$EXP_NAME_2/checkpoints/20000.pth" \
        --hair_conf_path "$PROJECT_DIR/src/arguments/hair_strands_textured.yaml" \
        --lambda_dmask 0.1 --lambda_dorient 0.1 --lambda_dsds 0.01 \
        --load_synthetic_rgba --load_synthetic_geom --binarize_masks --iteration_data 30000 \
        --position_lr_init 0.0000016 --position_lr_max_steps 10000 \
        --trainable_cameras --trainable_intrinsics --use_barf \
        --iterations 10000 --port "800$GPU"
else
    echo "Skipping train_strands.py: $DATA_PATH/curves_reconstruction/$EXP_NAME_3/checkpoints/10000.pth already exists"
fi

# rm -rf "$DATA_PATH/3d_gaussian_splatting/$EXP_NAME_1/train_cropped"

##################
# VISUALIZATIONS #
##################

# Export the resulting strands as pkl and ply
if [ ! -f "$DATA_PATH/curves_reconstruction/$EXP_NAME_3/exported_curves.pkl" ]; then
    echo "Exporting strands..."
    conda activate gaussian_splatting_hair && cd $PROJECT_DIR/src/preprocessing
    CUDA_VISIBLE_DEVICES="$GPU" python export_curves.py \
        --data_dir $DATA_PATH --model_name $EXP_NAME_3 --iter 10000 \
        --flame_mesh_path "$DATA_PATH/flame_fitting/$EXP_NAME_1/stage_3/mesh_final.obj" \
        --scalp_mesh_path "$DATA_PATH/flame_fitting/$EXP_NAME_1/scalp_data/scalp.obj" \
        --hair_conf_path "$PROJECT_DIR/src/arguments/hair_strands_textured.yaml"
else
    echo "Skipping export_curves.py: $DATA_PATH/curves_reconstruction/$EXP_NAME_3/exported_curves.pkl already exists"
fi

# Render the visualizations
if [ ! -f "$DATA_PATH/video.mp4" ]; then
    echo "Rendering visualizations..."
    conda activate gaussian_splatting_hair && cd $PROJECT_DIR/src/postprocessing
    CUDA_VISIBLE_DEVICES="$GPU" python render_video.py \
        --blender_path "$BLENDER_DIR" --input_path "$DATA_PATH" \
        --exp_name_1 "$EXP_NAME_1" --exp_name_3 "$EXP_NAME_3"
else
    echo "Skipping render_video.py: $DATA_PATH/video.mp4 already exists"
fi

# Render the strands
if [ ! -d "$DATA_PATH/rendered_strands" ]; then
    echo "Rendering strands..."
    conda activate gaussian_splatting_hair && cd $PROJECT_DIR/src
    CUDA_VISIBLE_DEVICES="$GPU" python render_strands.py \
        -s $DATA_PATH --data_dir "$DATA_PATH" --data_device 'cpu' --skip_test \
        -m "$EXP_PATH_1" --iteration 30000 \
        --flame_mesh_dir "$DATA_PATH/flame_fitting/$EXP_NAME_1" \
        --model_hair_path "$DATA_PATH/curves_reconstruction/$EXP_NAME_3" \
        --hair_conf_path "$PROJECT_DIR/src/arguments/hair_strands_textured.yaml" \
        --checkpoint_hair "$DATA_PATH/strands_reconstruction/$EXP_NAME_2/checkpoints/20000.pth" \
        --checkpoint_curves "$DATA_PATH/curves_reconstruction/$EXP_NAME_3/checkpoints/10000.pth" \
        --pointcloud_path_head "$EXP_PATH_1/point_cloud/iteration_30000/raw_point_cloud.ply" \
        --interpolate_cameras
else
    echo "Skipping render_strands.py: $DATA_PATH/rendered_strands already exists"
fi

# Convert strands to ABC
./blender-3.6.13-linux-x64/blender -b -P ./HAAR/src/utils/export_for_unreal.py

# # Make the video
# conda activate gaussian_splatting_hair && cd $PROJECT_DIR/src/postprocessing
# CUDA_VISIBLE_DEVICES="$GPU" python concat_video.py \
#     --input_path "$DATA_PATH" --exp_name_3 "$EXP_NAME_3"