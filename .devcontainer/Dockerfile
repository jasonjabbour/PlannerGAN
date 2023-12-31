# Use the official Ubuntu 22.04 as base image
FROM ubuntu:22.04

# Set environment variables to avoid interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Ensure the universe repository is available
RUN apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository universe

# Update the package list
RUN apt-get update

# Install necessary tools
RUN apt-get install -y \
    curl \
    git \
    make \
    build-essential \
    libssl-dev \
    zlib1g-dev \
    libbz2-dev \
    libreadline-dev \
    libsqlite3-dev \
    wget \
    llvm \
    libncurses5-dev \
    libncursesw5-dev \
    xz-utils \
    tk-dev \
    libxml2-dev \
    libxmlsec1-dev \
    libffi-dev \
    liblzma-dev \
    libxkbcommon-x11-0 \
    lsb-release \
    gnupg2 \
    python3-pip

# Add ROS 2 repository key
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -

# Add ROS 2 repository to the sources list
RUN echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# Update the package list to include the ROS 2 repository
RUN apt-get update

# Install ROS 2 Humble packages
RUN apt-get install -y \
    ros-humble-desktop \
    ros-humble-moveit

# Install vcstool and rosdep
RUN apt-get install -y \
    python3-vcstool \
    python3-rosdep

# Initialize rosdep (only once)
RUN if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then rosdep init; fi && rosdep update

# Source ROS 2 setup.bash in the bashrc file so ROS environment variables are automatically set
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Install additional ROS 2 dependencies and tools
RUN apt-get install -y \
    python3-rosinstall-generator \
    python3-wstool \
    python3-colcon-common-extensions

# # Install Gazebo
RUN apt-get install -y gazebo

# # Install ROS 2 packages for Gazebo integration
RUN apt-get install -y ros-humble-gazebo-ros-pkgs

# Install the tree command
RUN apt-get install -y tree

# Create the workspace directory
RUN mkdir -p /tmp/PlannerGAN_ws/src

# Expose a shell when the container is run
CMD ["/bin/bash"]
