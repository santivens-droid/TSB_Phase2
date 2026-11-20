#!/bin/bash

# HoloOcean Package Installation Script
# This script installs the HoloOcean package

# Change to the holoocean_client directory
cd /ws/holoocean

# Define the target directory
TARGET_DIR="/ws/holoocean/Packages/2.0.1/worlds/TSB/"
PACKAGE_FILE="/ws/holoocean/Package.zip"

# Check if Package.zip exists
if [ ! -f "$PACKAGE_FILE" ]; then
    echo "Error: $PACKAGE_FILE does not exist!"
    exit 1
fi

# Remove the target directory if it exists
echo "Removing existing target directory if it exists..."
rm -rf "$TARGET_DIR"

# Create the target directory
echo "Creating directory structure..."
mkdir -p "$TARGET_DIR"

# Unzip Package.zip to the target directory
echo "Unzipping Package.zip to world directory..."
unzip -o "$PACKAGE_FILE" -d "$TARGET_DIR"

# Make all files executable
echo "Making extracted files executable..."
find "$TARGET_DIR" -type f -exec chmod +x {} \;

# Check HoloOcean packages
python3 -c "import holoocean; print(holoocean.installed_packages())"

# Return to workspace root
cd /ws
