#!/bin/bash

# Find all png files in assets/$1 directories
# take dir from $1 as it is a file originally
INPUT_DIR=$(dirname "$1")
echo "Input directory: $INPUT_DIR"
files=$(find ./$INPUT_DIR -type f -iname "*.png")
echo "Found files: $files"

# filter files to only take front.png
files=$(echo "$files" | grep "front.png")

# Path to Conda installation
CONDA_PATH="/home/pkw/miniconda3"

# Input and output arguments from Makefile
INPUT_IMAGE="$1"
OUTPUT_DIR="$2"
export CUDA_VISIBLE_DEVICES="0"

# Copy all PNG files to the output directory
mkdir -p "$OUTPUT_DIR"
for file in $files; do
    if [ -f "$file" ]; then
        mkdir -p "$OUTPUT_DIR/img"
        cp "$file" "$OUTPUT_DIR/img/$(basename $file)"
        echo "Copied: $file → $OUTPUT_DIR/img/$(basename $file)"
    fi
done

# Source Conda initialization and run the hairstep commands

source "$CONDA_PATH/etc/profile.d/conda.sh"
conda activate hairstep
cd HairStep
python -m scripts.img2hairstep --root_real_imgs="../$OUTPUT_DIR"
python scripts/get_lmk.py --root_real_imgs="../$OUTPUT_DIR"
python -m scripts.opt_cam --root_real_imgs="../$OUTPUT_DIR"
python -m scripts.recon3D --root_real_imgs="../$OUTPUT_DIR"
conda deactivate