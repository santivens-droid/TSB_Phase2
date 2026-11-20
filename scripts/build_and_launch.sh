#!/bin/bash

# Build and launch the ROS system in one command

echo "=== ROS Build and Launch ==="

cd /ws
chmod +x scripts/*.sh

echo "Building workspace..."
./scripts/build_workspace.sh
echo ""

echo "Launching ROS ..."
./scripts/launch_workspace.sh
