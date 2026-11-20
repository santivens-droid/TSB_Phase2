# TSB Autonomous Systems Recruitment - Phase 2

This is the codebase for the **second phase** of the Técnico Solar Boat Autonomous Systems recruitment project. This repository provides a complete development environment using our work-in-progress simulator.

## Overview

This project is built on a **Docker/DevContainer** architecture, similar to the setup used in our main autonomous stack. This approach ensures a consistent development environment across different operating systems and simplifies dependency management.

The repository includes:
- **HoloOcean Integration**: Our new work-in-progress simulator for realistic marine environment testing
- **Multiple Vessel Models**: URDF descriptions for different vessels (Otter USV and SP01)
- **ROS 2 Humble**: Complete ROS 2 workspace with custom packages for simulation, control, and visualization
- **Foxglove Integration**: Modern visualization and debugging tools via Foxglove Bridge

## What is Docker and DevContainers?

**Docker** is a containerization platform that packages applications and their dependencies into isolated containers. Think of it as a lightweight virtual machine that ensures everyone runs the exact same environment, regardless of their host operating system.

**DevContainers** (Development Containers) are a VS Code feature that allows you to develop inside a Docker container. This means:
- All dependencies are pre-installed and configured
- You get the same development environment as your teammates
- Your host system remains clean - all tools run inside the container
- You can use VS Code normally, but everything executes inside the container

## Getting Started

### Prerequisites

- **Visual Studio Code** with the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
- **Docker Desktop** installed and running
- **GPU (NVIDIA recommended)** with drivers installed (recommended but not needed. Select non-NVIDIA DevContainer if unavailable)
- **Foxglove Studio** client ([download here](https://foxglove.dev/download))

> **Note**: The simulator doesn't work on Windows. Docker in Windows uses WSL2, which has an incomplete Vulkan implementation that prevents the simulator from running correctly.
> **Note**: The simulator doesn't work on ARM architectures due to Unreal Engine building limitations.
> **Note**: This setup has only been tested on **Linux x86_64 systems with NVIDIA GPU OR AMD GPU**.

> **Note**: The simulator requires Vulkan support and is currently only supported on **Linux x86_64 systems with NVIDIA or AMD GPUs**. It does not work on ARM architectures due to Unreal Engine building limitations. It does not work on Windows due to WSL's incomplete Vulkan implementation. We will try to provide a remote option for other OS users in the future.
Please contact us if you encounter any issues or need assistance with the setup.

### Opening the Workspace in DevContainer

1. Open this folder in VS Code
2. VS Code will detect the `.devcontainer` configuration and prompt you to "Reopen in Container"
   - If not prompted, press `F1` and select: `Dev Containers: Reopen in Container`
3. Select the appropriate devcontainer configuration: **NVIDIA GPU** (recommended for simulation performance) or **Non-NVIDIA GPU**
4. VS Code will build the Docker image and start the container (this may take several minutes on first run)
5. Once complete, you'll be developing inside the container with all dependencies ready!

The DevContainer automatically:
- Installs ROS 2 Humble
- Sets up all workspace dependencies
- Installs the HoloOcean simulator
- Configures GPU acceleration for simulation
- Mounts your workspace and Git configuration

### Running Multiple Containers on the Same Machine

When running multiple containers on the same machine (common when using a remote server), you need to modify `docker-compose.yml` to avoid conflicts:

**Option 1: Privileged Container with Host Networking**
- Uncomment the privileged settings: `privileged: true`, `network_mode: host`, `ipc: host`, `pid: host`
- Change `ROS_DOMAIN_ID` to a unique value (e.g., 31, 32, etc.) to isolate ROS 2 networks

**Option 2: Unprivileged Container**
- Keep the container unprivileged (comment out the privileged settings)
- Change the Foxglove port mapping to a unique external port (e.g., `10001:8765`)
- If using VSCode Remote - SSH, VS Code should automatically map a port for you. You can access using localhost:8765 or the mapped port shown in VS Code.

## ROS Workspace Structure

### Main Packages

#### **otter_description**
Provides the URDF model for the **Otter USV** (Unmanned Surface Vessel). This includes the robot's physical structure and frames.

#### **sp01_description**
Provides the URDF model for the **SP01 vessel**, Técnico Solar Boat's autonomous vessel.

#### **tsb_recruitment** (Metapackage)
A metapackage that bundles all recruitment-related packages:

- **tsb_recruitment_interfaces**: Custom ROS 2 message and service definitions
- **tsb_recruitment_sim**: HoloOcean simulator interface that bridges the maritime simulator with ROS 2
- **tsb_recruitment_bringup**: Launch files and configuration for starting different simulation scenarios
- **tsb_recruitment_teleop**: Teleoperation node for manual control of vessels using keyboard input
- **tsb_recruitment_datagenerator**: Publishes fake sensor data for testing visualization and dashboard development

## Available Scripts

All scripts are located in the `scripts/` directory and are **only available inside the Docker container**. Run them from the workspace root:

- **`./scripts/build_workspace.sh`**: Builds the entire ROS 2 workspace using colcon
- **`./scripts/clean_workspace.sh`**: Removes build artifacts and cleans the workspace
- **`./scripts/launch_workspace.sh`**: Launches the complete workspace (otter simulator + bridge)
- **`./scripts/launch_teleop.sh`**: Launches the teleoperation interface for manual vessel control
- **`./scripts/build_and_launch.sh`**: Builds the workspace and launches the system in one command
- **`./scripts/record_bag.sh <bagname>`**: Records all ROS topics to a bag file in MCAP format
- **`./scripts/install_holoocean.sh`**: Installs or updates the HoloOcean simulator (should not be needed since container runs it at start)

## Available VS Code Tasks

You can run pre-configured tasks via VS Code's Command Palette (Ctrl+Shift+P → Tasks: Run Task) or from the top menu: Terminal → Run Task.

- **Build and Launch**: Builds the workspace and launches the system
- **Build Workspace**: Builds all ROS 2 packages (default build task - `Ctrl+Shift+B`)
- **Clean Workspace**: Cleans build and install folders
- **Launch Workspace**: Starts the simulator and Foxglove bridge
- **Install Holoocean**: Installs/updates the HoloOcean simulator

## Launch Files

### Simulator Launches

#### Otter Simulator
```bash
# With viewport (default)
ros2 launch tsb_recruitment_bringup otter_simulator.launch.py

# Headless (no viewport window)
ros2 launch tsb_recruitment_bringup otter_simulator_headless.launch.py
```
Launches the HoloOcean simulator with the **Otter USV** and Foxglove Bridge.

The headless version runs without the viewport window, suitable for headless environments, used for remote development.

> **Note for Remote Development**: The simulator viewport (GUI) cannot be displayed over SSH connections because SSH does not support Vulkan forwarding. When developing remotely (e.g., via VS Code Remote-SSH), you must use the headless launch files. All simulation features work identically - only the visual viewport window is disabled. You can still visualize data through Foxglove Studio.

#### SP01 Simulator
```bash
# With viewport (default)
ros2 launch tsb_recruitment_bringup sp01_simulator.launch.py

# Headless (no viewport window)
ros2 launch tsb_recruitment_bringup SP01_simulator_headless.launch.py
```
Launches the HoloOcean simulator with the **SP01 vessel** and Foxglove Bridge.

The headless version runs without the viewport window, suitable for headless environments, used for remote development.

> **Note for Remote Development**: The simulator viewport (GUI) cannot be displayed over SSH connections because SSH does not support Vulkan forwarding. When developing remotely (e.g., via VS Code Remote-SSH), you must use the headless launch files. All simulation features work identically - only the visual viewport window is disabled. You can still visualize data through Foxglove Studio.

### Data Generator (No Simulator)
```bash
ros2 launch tsb_recruitment_bringup sp01_datagenerator.launch.py
```
Launches a node that publishes **fake sensor data** for SP01 and the Foxglove Bridge.

## Teleoperation

Both the Otter and SP01 simulators start with an **internal PID controller** that maintains the vessel's position. To manually control the vessels:

1. Launch either the Otter or SP01 simulator (see above)
2. In a **new terminal**, run:
   ```bash
   ./scripts/launch_teleop.sh
   ```
3. Use the following keys to control the vessel:
- **W**: Increase forward speed (incrementally raises the commanded forward velocity)
- **S**: Decrease forward speed (incrementally lowers the commanded forward velocity)
- **A**: Increase yaw to the left (incrementally commands more left rotation)
- **D**: Increase yaw to the right (incrementally commands more right rotation)

These keys adjust the vessel's commanded speed/yaw persistently (they change the setpoint rather than providing a momentary input).

## Foxglove Studio

[Foxglove Studio](https://foxglove.dev/) is a modern visualization and debugging platform for robotics. It is what we use in our main autonomous stack instead of RViz.

### What is the Foxglove Bridge?

The Foxglove Bridge is provided as a ROS 2 node (foxglove_bridge) that runs alongside your ROS 2 system inside the container and exposes a WebSocket server so Foxglove Studio can connect and exchange messages with ROS 2.

The bridge is automatically launched with all simulator launch files.

### Connecting Foxglove to the Simulator

1. **Install Foxglove Studio** on your host machine (not inside Docker): [Download here](https://foxglove.dev/download)
2. **Launch** one of the simulator launch files (the Foxglove Bridge starts automatically)
3. **Open Foxglove Studio** and click "Open Connection"
4. **Select "Foxglove Websocket"**
5. **Enter the WebSocket URL**: `ws://localhost:10000` (Or the mapped port if you changed it)
6. **Click "Open"**

You should now see live data from the simulator! You can add panels to visualize 3D views, camera feeds, sensor data, and more.

## Working with ROS Bags

### Recording a Bag

To record all topics to a bag file in MCAP format:

```bash
/ws/scripts/record_bag.sh <bagname>
```

**Example:**
```bash
/ws/scripts/record_bag.sh my_test_run
```

This will:
- Record all active topics (`-a` flag)
- Save the bag in MCAP format (modern, efficient format)
- Store the bag in `/ws/bags/<bagname>/`
- Press `Ctrl+C` to stop recording

### Playing Back a Bag

To replay a recorded bag with Foxglove visualization:

```bash
ros2 launch tsb_recruitment_bringup bag_playback.launch.py bag_path:=/ws/bags/<bagname>
```

**Example:**
```bash
ros2 launch tsb_recruitment_bringup bag_playback.launch.py bag_path:=/ws/bags/sp01_simulator1
```

This launch file will:
- Play back the specified bag
- Start the Foxglove Bridge for visualization
- Allow you to connect Foxglove Studio (see Foxglove section above)

**Additional playback options:**
```bash
# Play at 2x speed
ros2 launch tsb_recruitment_bringup bag_playback.launch.py bag_path:=/ws/bags/my_test_run rate:=2.0

# Loop the bag continuously
ros2 launch tsb_recruitment_bringup bag_playback.launch.py bag_path:=/ws/bags/my_test_run loop:=true
```

## Development Workflow

1. **Make changes** to the code in VS Code (running inside the container)
2. **Build the workspace**:
   ```bash
   ./scripts/build_workspace.sh
   ```
   Or use the VS Code task: `Ctrl+Shift+B`
3. **Source the workspace**:
    ```bash
    ./scripts/source_workspace.sh
    ```
4. **Launch** the appropriate launch file for testing
5. **Connect Foxglove Studio** to visualize and debug

## Troubleshooting

### Foxglove won't connect
- Verify the bridge URL is exactly: `ws://localhost:8765`
- Ensure the simulator/launch file is running

### Build errors
- Try cleaning and rebuilding: `./scripts/clean_workspace.sh && ./scripts/build_workspace.sh`

## Support

For questions or issues, contact us.

---

**Good luck with your project! 🚤**