FROM osrf/ros:humble-desktop

# Create non-root user
ARG USERNAME=skibidi
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install additional ROS packages, dependencies, and Git
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    openssh-client \
    ros-humble-rqt-plot \
    ros-humble-rqt-graph \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Switch to non-root user
USER $USERNAME
WORKDIR /home/$USERNAME

# Source ROS environment in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /home/$USERNAME/ros_ws/install/setup.bash ]; then source /home/$USERNAME/ros_ws/install/setup.bash; fi" >> /home/$USERNAME/.bashrc

# Ensure SSH directory exists
RUN mkdir -p /home/$USERNAME/.ssh && chmod 700 /home/$USERNAME/.ssh

# Automatically add GitHub to known hosts to prevent SSH prompt
RUN ssh-keyscan github.com >> /home/$USERNAME/.ssh/known_hosts

CMD ["bash"]