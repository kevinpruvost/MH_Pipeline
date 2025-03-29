#!/bin/bash

# Find all mp4 files in assets/$1 directories
# take dir from $1 as it is a file originally
INPUT_DIR=$(dirname "$1")
echo "Input directory: $INPUT_DIR"
files=$(find ./$INPUT_DIR -type f -iname "*.mp4")
echo "Found files: $files"

# Path to Conda installation
CONDA_PATH="/home/pkw/miniconda3"

# Input and output arguments from Makefile
INPUT_IMAGE="$1"
OUTPUT_DIR="$2"
#if gpu specified, use it
if [ -z "$3" ]; then
    export CUDA_VISIBLE_DEVICES="0"
else
    export CUDA_VISIBLE_DEVICES="$3"
fi

# Copy all PNG files to the output directory
mkdir -p "$OUTPUT_DIR"
for file in $files; do
    if [ -f "$file" ]; then
        mkdir -p "$OUTPUT_DIR/img"
        cp "$file" "$OUTPUT_DIR/$(basename $file)"
        echo "Copied: $file → $OUTPUT_DIR/$(basename $file)"
    fi
done

# Source Conda initialization and run the hairstep commands

source "$CONDA_PATH/etc/profile.d/conda.sh"
cd GaussianHaircut
echo
#bash run.sh "../$OUTPUT_DIR/"