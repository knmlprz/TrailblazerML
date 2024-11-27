#!/bin/bash

# Target directory for downloads
TARGET_DIR="../src/gazebo_viz/meshes/"

# Check if the target directory exists, if not, create it
if [ ! -d "$TARGET_DIR" ]; then
    mkdir -p "$TARGET_DIR"
fi

# Function to download a file from Google Drive to the target directory
download_from_gdrive() {
    FILE_ID=$1
    FILE_NAME=$2
    wget --no-check-certificate "https://drive.google.com/uc?export=download&id=${FILE_ID}" -O "${TARGET_DIR}${FILE_NAME}"
}

# Download the first file
download_from_gdrive "15zxd2lkHrjEpR9u3mbJMq8X4JvI5K9S1" "body.stl"

# Download the second file
download_from_gdrive "16o9_4iDnvVWLI9OwD-syQ8tkgXyOhGB3" "wheel_left.stl"

# Download the third file
download_from_gdrive "1eaZmvw02DhXmTS0OnvaBMUuCm69O_Pz8" "wheel_right.stl"