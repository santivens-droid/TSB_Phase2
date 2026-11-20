# Source workspace environment script

# Source the ROS environment
source /opt/ros/humble/setup.bash

# Source the local workspace
if [ -d "/ws/install" ]; then
    source /ws/install/setup.bash
else
    echo "Error: Workspace not built. Run ./scripts/build_workspace.sh first."
    return 1 2>/dev/null || exit 1
fi