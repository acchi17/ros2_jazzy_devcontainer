# Download base imgae if it doesn't exist locally
FROM ros:jazzy

# Install required packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    cmake \
    gdb \
    git \
    locales \
    python3-colcon-common-extensions \
    python3-flake8 \
    python3-pip \
    python3-pytest-cov \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    sudo \
    vim \
    wget \
    ros-jazzy-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# Locale settings
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LC_ALL en_US.UTF-8

# Initialize rosdep
# * This command requires root privileges, so execute it before USER $USERNAME
# "|| true" is added to prevent build failure if already initialized or in CI environments
RUN rosdep init || true

# Add user & grant sudo privileges
ARG USERNAME=vscode
ARG USER_UID=7777
ARG USER_GID=7777
ARG USER_PASSWORD=vscode
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME:$USER_PASSWORD" | chpasswd \
    && usermod -aG sudo $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Create and set up ROS2 workspace
RUN mkdir -p /home/$USERNAME/ros2_projects/ros2_ws/src \
    && echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc

# Set up colcon auto-completion, command history size, and history file size
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc \
    && echo "export HISTSIZE=10000" >> /home/$USERNAME/.bashrc \
    && echo "export HISTFILESIZE=10000" >> /home/$USERNAME/.bashrc

# Set default user
USER $USERNAME

# Set working directory
WORKDIR /home/$USERNAME/ros2_projects

CMD ["bash"]
