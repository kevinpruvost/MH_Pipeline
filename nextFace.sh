#!/bin/bash

# Path to Conda installation
CONDA_PATH="/home/pkw/miniconda3"

# Input and output arguments from Makefile
INPUT_IMAGE="$1"
OUTPUT_DIR="$2"

# If INPUT_IMAGE is a directory, then search for front.png inside it
if [ -d "$INPUT_IMAGE" ]; then
    INPUT_IMAGE=$(find "$INPUT_IMAGE" -type f -name "front.png" | head -n 1)
    if [ -z "$INPUT_IMAGE" ]; then
        echo "Error: front.png not found in $INPUT_IMAGE"
        exit 1
    fi
fi

# Source Conda initialization and run the NextFace command
source "$CONDA_PATH/etc/profile.d/conda.sh"
conda activate faceNext
cd NextFace
python optimizer.py --input "../$INPUT_IMAGE" --output "../$OUTPUT_DIR" --config optimConfig.ini
conda deactivate