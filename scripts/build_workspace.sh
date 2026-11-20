#!/bin/bash

# Build the ROS workspace

echo "=== Building ROS Workspace ==="

cd /ws
source /opt/ros/humble/setup.bash

# Source existing workspace if available (for incremental builds)
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
fi

# Build the workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON

# Source the built workspace
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "Build complete!"
else
    echo "Build failed - install/setup.bash not found"
    exit 1
fi
