FROM osrf/ros:humble-desktop
SHELL ["/bin/bash", "-c"]

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Install Node 22 and Codex
RUN apt-get update && apt-get install -y \
    curl \
    ca-certificates \
    gnupg \
    && curl -fsSL https://deb.nodesource.com/setup_22.x | bash - \
    && apt-get install -y nodejs \
    && npm install -g @openai/codex \
    && rm -rf /var/lib/apt/lists/*
    

# Install system dependencies
RUN apt-get update && apt-get install -y \
    libboost-system-dev \
    libboost-thread-dev \
    python3-pip \
    python3-tk \
    qtwayland5 \
    ros-humble-mavros ros-humble-mavros-extras ros-humble-mavros-msgs \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user so bind-mounted workspace files are not owned by root.
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# Install python dependencies
RUN pip3 install \
    "numpy<1.25.0" \
    gtsam \
    matplotlib \
    scikit-learn \
    scipy \
    shapely \
    tqdm

# Install and build the Oculus ROS2 workspace
WORKDIR /ros2_ws

# Clone the repo into the 'src' folder
RUN git clone https://github.com/ENSTABretagneRobotics/oculus_ros2.git src/oculus_ros2
RUN git clone https://github.com/ENSTABretagneRobotics/oculus_driver.git src/oculus_driver
RUN git clone https://github.com/paagutie/dvl_msgs.git src/dvl_msgs

# Source the ROS environment and build the workspace
RUN source /opt/ros/humble/setup.bash && colcon build

# Add the workspace setup for both root and the dev container user.
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc \
    && echo "source /ros2_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc \
    && chown ${USERNAME}:${USERNAME} /home/${USERNAME}/.bashrc
