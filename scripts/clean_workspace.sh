#!/bin/bash

# Clean the ROS workspace by removing build, install, and log directories

echo "=== Cleaning ROS Workspace ==="

cd /ws

# Check if directories exist and remove them
if [ -d "build" ]; then
    echo "Removing build directory..."
    rm -rf build
fi

if [ -d "install" ]; then
    echo "Removing install directory..."
    rm -rf install
fi

if [ -d "log" ]; then
    echo "Removing log directory..."
    rm -rf log
fi

echo "Workspace cleaned successfully!"
echo "You can now run './scripts/build_workspace.sh' for a fresh build."
