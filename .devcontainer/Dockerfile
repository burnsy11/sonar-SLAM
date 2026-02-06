FROM osrf/ros:humble-desktop
SHELL ["/bin/bash", "-c"]

# Install system dependencies
# Added 'python3-pip' to fix the error
RUN apt-get update && apt-get install -y \
    libboost-system-dev \
    libboost-thread-dev \
    python3-pip \
    python3-tk \
    x11-apps \
    ros-humble-mavros ros-humble-mavros-extras ros-humble-mavros-msgs \
    && rm -rf /var/lib/apt/lists/*

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

# Add the workspace setup to the .bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc