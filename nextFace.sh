#!/bin/bash

# Path to Conda installation
CONDA_PATH="/home/pkw/miniconda3"

# Input and output arguments from Makefile
INPUT_IMAGE="$1"
OUTPUT_DIR="$2"

# Source Conda initialization and run the NextFace command
source "$CONDA_PATH/etc/profile.d/conda.sh"
conda activate faceNext
cd NextFace
python optimizer.py --input "../$INPUT_IMAGE" --output "../$OUTPUT_DIR" --config optimConfig.ini
conda deactivate