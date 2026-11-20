#!/bin/bash

# Launch the teleoperation node

echo "=== Launching Teleoperation Node ==="

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

echo "Starting teleoperation node..."
echo ""

# Launch the teleop node
exec ros2 run tsb_recruitment_teleop TeleopNode --ros-args -p control_mode:=surge_yaw -p max_surge:=2.0 -p surge_increment:=0.1 -p yaw_increment:=5.0 -r command:=auv0/command