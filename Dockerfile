FROM osrf/ros:humble-desktop

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install basic dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    wget \
    curl \
    vim \
    bash-completion \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Install additional ROS2 dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rviz2 \
    ros-humble-xacro \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    && rm -rf /var/lib/apt/lists/*

# Create user skibidi
RUN useradd -m skibidi -s /bin/bash && \
    echo "skibidi:skibidi" | chpasswd && \
    adduser skibidi sudo && \
    echo "skibidi ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Setup ROS2 environment for skibidi
RUN echo "source /opt/ros/humble/setup.bash" >> /home/skibidi/.bashrc

# Create workspace directory structure in skibidi's home
RUN mkdir -p /home/skibidi/ros_ws/src && \
    chown -R skibidi:skibidi /home/skibidi/ros_ws

# Switch to the skibidi user
USER skibidi

# Set working directory to skibidi's workspace
WORKDIR /home/skibidi/ros_ws

CMD ["bash"]