# Base ROS2 Humble image - includes core ROS2 packages and desktop tools
FROM osrf/ros:humble-desktop-full

ARG WORKSPACE=humble_dev_ws
WORKDIR /root/$WORKSPACE

# GPU support for both NVIDIA and Mesa (Intel/AMD)
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Wayland and Qt configuration for Fedora 43
ENV QT_QPA_PLATFORM=wayland
ENV GDK_BACKEND=wayland
ENV CLUTTER_BACKEND=wayland
ENV SDL_VIDEODRIVER=wayland
ENV XDG_RUNTIME_DIR=/tmp
ENV EDITOR=nano

# Fallback to X11 if Wayland is not available
ENV QT_X11_NO_MITSHM=1

# Install development tools, ROS2 packages, and Wayland support
RUN apt-get update && apt-get install -y \
    cmake \
    curl \
    gazebo \
    libglu1-mesa-dev \
    libwayland-dev \
    libxkbcommon-dev \
    mesa-utils \
    nano \
    python3-pip \
    python3-pydantic \
    qtwayland5 \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-joint-state-publisher \
    ros-humble-robot-localization \
    ros-humble-plotjuggler-ros \
    ros-humble-robot-state-publisher \
    ros-humble-ros2bag \
    ros-humble-rosbag2-storage-default-plugins \
    ros-humble-rqt-tf-tree \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-slam-toolbox \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-msgs \
    ros-humble-twist-mux \
    ros-humble-usb-cam \
    ros-humble-xacro \
    ruby-dev \
    rviz \
    tmux \
    wayland-protocols \
    wget \
    xorg-dev \
    zsh

# Fix setuptools version for ROS2 compatibility
RUN pip3 install setuptools==58.2.0

# Install urdf-viz - a tool for visualizing URDF robot models
RUN wget https://github.com/openrr/urdf-viz/releases/download/v0.38.2/urdf-viz-x86_64-unknown-linux-gnu.tar.gz && \
    tar -xvzf urdf-viz-x86_64-unknown-linux-gnu.tar.gz -C /usr/local/bin/ && \
    chmod +x /usr/local/bin/urdf-viz && \
    rm -f urdf-viz-x86_64-unknown-linux-gnu.tar.gz

# Install ZSH with helpful plugins for better terminal experience
# - git: adds git aliases and branch info
# - zsh-autosuggestions: suggests commands as you type
# - zsh-completions: additional command completions
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.2/zsh-in-docker.sh)" -- \
    -p git \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-completions

# Install tmuxinator - manages tmux sessions from YAML config files
RUN gem install tmuxinator && \
    wget https://raw.githubusercontent.com/tmuxinator/tmuxinator/master/completion/tmuxinator.zsh -O /usr/local/share/zsh/site-functions/_tmuxinator

# Clean up to reduce image size
RUN apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Configure ZSH shell environment
RUN echo "export DISABLE_AUTO_TITLE=true" >> /root/.zshrc
RUN echo 'LC_NUMERIC="en_US.UTF-8"' >> /root/.zshrc

# Source ROS2 Humble and Gazebo environments automatically
RUN echo "source /opt/ros/humble/setup.zsh" >> /root/.zshrc
RUN echo "source /usr/share/gazebo/setup.sh" >> /root/.zshrc

# Helpful aliases for ROS2 development (for beginners):
# rosdi  - Install dependencies for your ROS2 packages
# cbuild - Build your ROS2 workspace with symlink install (faster rebuilds)
# ssetup - Source your workspace after building
# cyclone/fastdds - Switch between DDS implementations (communication middleware)
RUN echo 'alias rosdi="rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y"' >> /root/.zshrc
RUN echo 'alias cbuild="colcon build --symlink-install"' >> /root/.zshrc
RUN echo 'alias ssetup="source ./install/local_setup.zsh"' >> /root/.zshrc
RUN echo 'alias cyclone="export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"' >> /root/.zshrc
RUN echo 'alias fastdds="export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"' >> /root/.zshrc

# Set CycloneDDS as default (better performance for most use cases)
RUN echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> /root/.zshrc

# Enable bash-style completion for ROS2 and Colcon commands
RUN echo "autoload -U bashcompinit" >> /root/.zshrc
RUN echo "bashcompinit" >> /root/.zshrc
RUN echo 'eval "$(register-python-argcomplete3 ros2)"' >> /root/.zshrc
RUN echo 'eval "$(register-python-argcomplete3 colcon)"' >> /root/.zshrc

# Start tmuxinator session on container launch
CMD [ "tmuxinator", "start", "-p", "/root/.session.yml" ]
