#!/bin/bash

# Find all png files in assets/$1 directories
# take dir from $1 as it is a file originally
INPUT_DIR="$1"

# Path to Conda installation
CONDA_PATH="/home/pkw/miniconda3"

# Input and output arguments from Makefile
INPUT_IMAGE="$1"
OUTPUT_DIR="$2"
export CUDA_VISIBLE_DEVICES="0"

# Get absolute paths for input and output directories
RAW_INPUT_DIR=$(realpath "$INPUT_DIR" 2>/dev/null || echo "$INPUT_DIR")
RAW_OUTPUT_DIR=$(realpath "$OUTPUT_DIR" 2>/dev/null || echo "$OUTPUT_DIR")

echo "Input directory: $RAW_INPUT_DIR"
echo "Output directory: $RAW_OUTPUT_DIR"

# Source Conda initialization and run the hairstep commands

source "$CONDA_PATH/etc/profile.d/conda.sh"
conda activate TDDFAV3
cd 3DDFA
echo "Running 3DDFAV3..."
python demo.py --inputpath "$RAW_INPUT_DIR" --savepath "$RAW_OUTPUT_DIR" --device cuda --iscrop 1 --detector retinaface --ldm68 1 --ldm106 1 --ldm106_2d 1 --ldm134 1 --seg_visible 1 --seg 1 --useTex 1 --extractTex 1 --backbone resnet50
conda deactivate