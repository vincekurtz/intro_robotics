# Use a pre-built ROS image with Ubuntu 24.04 and ROS2 Jazzy
FROM osrf/ros:jazzy-desktop

# Run as a non-root user. This user account is pre-created in the base image.
USER ubuntu
WORKDIR /home/ubuntu

# Install Turtlebot3 packages
RUN mkdir -p ~/turtlebot3_ws/src
WORKDIR /home/ubuntu/turtlebot3_ws/src
RUN git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git
RUN git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
RUN git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git 
WORKDIR /home/ubuntu/turtlebot3_ws
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install

# Install SLAM and navigation packages
USER root
RUN apt-get update && apt-get install -y \
    ros-jazzy-cartographer \
    ros-jazzy-cartographer-ros \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup
USER ubuntu

# Configure the environment. Note that ROS_DOMAIN_ID must match the robot.
WORKDIR /home/ubuntu
RUN echo 'source /home/ubuntu/turtlebot3_ws/install/setup.bash' >> /home/ubuntu/.bashrc
RUN echo 'export ROS_DOMAIN_ID=30' >> /home/ubuntu/.bashrc
RUN echo 'export TURTLEBOT3_MODEL=burger' >> /home/ubuntu/.bashrc
RUN echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> /home/ubuntu/.bashrc
