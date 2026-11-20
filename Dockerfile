# ---------------- Isaac ROS base (ROS 2 Humble) ----------------
FROM  ros:humble-ros-base

ARG TARGETARCH
ARG USERNAME=ros
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV ROS_DOMAIN_ID=30
ENV ROS_LOCALHOST_ONLY=true

# Core tools
RUN apt-get update && apt-get install -y \
    build-essential cmake git curl gnupg lsb-release unzip \
    python3-pip python3-vcstool python3-colcon-common-extensions \
    libgeographic-dev geographiclib-tools libpcl-dev \
    vim nano htop tree can-utils \
    && rm -rf /var/lib/apt/lists/*

# ROS packages
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    ros-humble-rviz2 \
    ros-humble-image-common ros-humble-cv-bridge \
    ros-humble-diagnostic-updater ros-humble-rosbridge-suite \
    ros-humble-pcl-conversions ros-humble-pcl-ros \
    ros-humble-foxglove-bridge \
    ros-humble-joint-state-publisher ros-humble-robot-state-publisher \
    ros-humble-tf2-tools ros-humble-serial-driver \
    && rm -rf /var/lib/apt/lists/*



# Create user
RUN useradd -G video -ms /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/99_$USERNAME \
    && chmod 0440 /etc/sudoers.d/99_$USERNAME

# Source environment
RUN echo '[ -f /ws/scripts/source_workspace.sh ] && source /ws/scripts/source_workspace.sh' >> /home/${USERNAME}/.bashrc

USER $USERNAME

# Setup HoloOcean packages path
ENV HOLODECKPATH=/ws/holoocean/Packages/

CMD ["/bin/bash"]
