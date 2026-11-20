#!/bin/bash

# Launch the ROS system

echo "=== Launching ROS System ==="

cd /ws
source /opt/ros/humble/setup.bash

# Source workspace
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "Workspace sourced successfully!"
else
    echo "ERROR: Workspace not built! Run build_workspace.sh first."
    exit 1
fi

echo "Starting ROS system..."
echo "Command: ros2 launch tsb_recruitment_bringup otter_simulator_headless.launch.py"
echo ""

# Launch the system
exec ros2 launch tsb_recruitment_bringup otter_simulator_headless.launch.py
