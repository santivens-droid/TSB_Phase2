#!/bin/bash

# Simple ROS 2 bag recording script with MCAP format
# Usage: ./record_bag.sh <bag_name>

# Check if bag name is provided
if [ -z "$1" ]; then
    echo "Error: Bag name is required"
    echo "Usage: $0 <bag_name>"
    echo "Example: $0 my_recording"
    exit 1
fi

BAG_NAME="$1"
BAGS_DIR="/ws/bags"
OUTPUT_PATH="${BAGS_DIR}/${BAG_NAME}"

# Create bags directory if it doesn't exist
mkdir -p "${BAGS_DIR}"

# Check if bag already exists
if [ -d "${OUTPUT_PATH}" ]; then
    echo "Warning: Bag directory '${OUTPUT_PATH}' already exists"
    read -p "Do you want to overwrite it? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Recording cancelled"
        exit 1
    fi
    rm -rf "${OUTPUT_PATH}"
fi

echo "========================================"
echo "Recording ROS 2 bag"
echo "========================================"
echo "Name:   ${BAG_NAME}"
echo "Path:   ${OUTPUT_PATH}"
echo "Format: MCAP"
echo "========================================"
echo "Press Ctrl+C to stop recording"
echo "========================================"
echo

# Record all topics with MCAP format
ros2 bag record -a \
    --storage mcap \
    -o "${OUTPUT_PATH}"

echo
echo "========================================"
echo "Recording stopped"
echo "Bag saved to: ${OUTPUT_PATH}"
echo "========================================"
