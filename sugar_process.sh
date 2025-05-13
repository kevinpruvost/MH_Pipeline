#!/bin/bash

# colmap_process.sh - Process a video file with COLMAP for a portable 3D reconstruction project with dense meshing

# Check if correct number of arguments are provided
if [ "$#" -ne 3 ]; then
    echo "Usage: $0 <input_video> <project_dir> <output_dir>"
    echo "Example: $0 ./assets/folder/raw.mp4 ./results/folder/sugar ./results/folder/sugar"
    exit 1
fi

INPUT_VIDEO=$1
PROJECT_DIR=$2  # This will be the single project folder (e.g., results/folder/colmap/)
OUTPUT_DIR=$3   # Should be the same as PROJECT_DIR for simplicity

# Convert INPUT_VIDEO to absolute path
ABS_INPUT_VIDEO=$(realpath "$INPUT_VIDEO" 2>/dev/null || echo "$INPUT_VIDEO")

# Create the project directory if it doesn't exist
mkdir -p "$PROJECT_DIR"

# Change to the project directory to ensure relative paths
cd "$PROJECT_DIR" || exit 1

echo "Processing $ABS_INPUT_VIDEO with SuGaR in project folder $PROJECT_DIR..."

# Define relative paths
IMAGES_DIR="input"
SPARSE_DIR="sparse/0"
DENSE_DIR="dense"
DATABASE_PATH="database.db"

# Extract frames
echo "Extracting frames to $IMAGES_DIR..."
mkdir -p "$IMAGES_DIR"
ffmpeg -i "$ABS_INPUT_VIDEO" -r 8 "$IMAGES_DIR/frame%d.jpg" 2>/dev/null || true

# Ensure the Conda environment activation works
source ~/miniconda3/etc/profile.d/conda.sh  # Adjust this path if using Anaconda or another install location
conda activate sugar
cd -
cd SuGaR
python gaussian_splatting/convert.py -s "../$PROJECT_DIR/"
python train_full_pipeline.py -s "../$PROJECT_DIR" -r "sdf" --high_poly True --export_obj True --gpu 3 --mesh_output_dir "../$PROJECT_DIR"
conda deactivate