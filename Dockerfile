# Use a pre-built ROS image with Ubuntu 24.04 and ROS2 Jazzy
FROM osrf/ros:jazzy-desktop

# Remove the default user 'ubuntu' to avoid potential conflicts
RUN deluser --remove-home ubuntu    
RUN rm -rf /home/ubuntu

# Run as a non-root user. The user and group IDs are created to match the host,
# so that files created in mounted volumes have the correct permissions.
ARG USERNAME=devuser
ARG UID=1000
ARG GID=${UID}

RUN groupadd -g ${GID} ${USERNAME} \
    && useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USERNAME} \
    && apt-get update && apt-get install -y sudo \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

WORKDIR /home/${USERNAME}
USER ${USERNAME}

# Install Turtlebot3 packages
RUN mkdir -p ~/ros2_ws/src
WORKDIR /home/${USERNAME}/ros2_ws/src
RUN git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git
RUN git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
RUN git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git 
WORKDIR /home/${USERNAME}/ros2_ws
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install

# Install SLAM and navigation packages
USER root
RUN apt-get update && apt-get install -y \
    ros-jazzy-cartographer \
    ros-jazzy-cartographer-ros \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup
USER ${USERNAME}

# Configure the environment. Note that ROS_DOMAIN_ID must match the robot.
WORKDIR /home/${USERNAME}
RUN echo "source /home/${USERNAME}/ros2_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc
RUN echo 'export ROS_DOMAIN_ID=30' >> /home/${USERNAME}/.bashrc
RUN echo 'export TURTLEBOT3_MODEL=burger' >> /home/${USERNAME}/.bashrc
RUN echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> /home/${USERNAME}/.bashrc
