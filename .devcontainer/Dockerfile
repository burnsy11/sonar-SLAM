FROM osrf/ros:humble-desktop
SHELL ["/bin/bash", "-c"]

# Install system dependencies
# Added 'python3-pip' to fix the error
RUN apt-get update && apt-get install -y \
    libboost-system-dev \
    libboost-thread-dev \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install python dependencies
# REMOVED conflicting ROS packages (rclpy, cv-bridge, etc.)
# REMOVED packages already included (numpy, opencv, etc.)
# Kept only the non-ROS, non-default packages.
RUN pip3 install \
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

# Source the ROS environment and build the workspace
RUN source /opt/ros/humble/setup.bash && colcon build

# Add the workspace setup to the .bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc