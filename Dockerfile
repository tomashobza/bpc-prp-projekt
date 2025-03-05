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

# Setup ROS2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc

# Create workspace directory structure
RUN mkdir -p /workspaces/ros_ws/src

# Set working directory
WORKDIR /workspaces/ros_ws

# # Add a script to source ROS environment on container start
# COPY entrypoint.sh /entrypoint.sh
# RUN chmod +x /entrypoint.sh

# ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]