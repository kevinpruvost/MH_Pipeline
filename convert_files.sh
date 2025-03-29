#!/bin/bash

# Find all JPG and JPEG files in assets/ directories
files=$(find ./assets -type f \( -iname "*.jpg" -o -iname "*.jpeg" -o -iname "*.JPG" \))

for file in $files; do
    if [ -f "$file" ]; then
        # Generate output filename by replacing .jpg or .jpeg with .png
        output="${file%.*}.png"

        # Convert JPG to PNG using ffmpeg
        ffmpeg -i "$file" "$output"

        echo "Converted: $file → $output"
        rm "$file"
    fi
done

# Convert MOV to MP4 and rename to raw.mp4
echo "Converting MOV to MP4..."
find ./assets -type f \( -iname "*.mov" -o -iname "*.MOV" \) | while read -r file; do
    if [ -f "$file" ]; then
        dir=$(dirname "$file")  # Get the directory of the file
        output="$dir/raw.mp4"   # Rename output to raw.mp4 in the same directory
        ffmpeg -i "$file" -c:v libx264 -crf 23 -preset fast -c:a aac -b:a 192k "$output"
        echo "Converted: $file → $output"
        rm "$file"  # Remove original MOV
    fi
done

echo "Conversion complete!"
