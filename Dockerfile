# Dockerfile for persistent ROS2 Humble dev container
FROM ubuntu:22.04

# Non-interactive mode for apt
ENV DEBIAN_FRONTEND=noninteractive

# Set up locale
RUN apt update && apt install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Install base tools
RUN apt update && apt install -y \
    curl gnupg2 lsb-release git build-essential vim wget sudo

# Add ROS2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'

# Install ROS2 Humble desktop (includes RViz, Gazebo)
RUN apt update && apt install -y ros-humble-desktop

# Add Gazebo (Ignition Fortress) apt repository for ROS2 Humble
RUN apt update && apt install -y lsb-release wget gnupg && \
    wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/gazebo-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list

# Add Ignition Gazebo (Fortress) and ROS–Gazebo bridge
RUN apt update && apt install -y \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    && rm -rf /var/lib/apt/lists/*

# Install TurtleBot3 packages and Gazebo ROS packages
RUN apt update && apt install -y \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-simulations \
    ros-humble-turtlebot3-msgs \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 Navigation (Nav2) packages
RUN apt update && apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*

# Common dev tools & visualization dependencies
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    git \
    wget \
    vim \
    x11-apps \
    tree \
    && rm -rf /var/lib/apt/lists/*

# Auto-source setup automatically on login
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Create workspace directory
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws

# Default command: open bash
CMD ["bash"]
