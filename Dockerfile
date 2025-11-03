# Use a pre-built ROS image with Ubuntu 24.04 and ROS2 Jazzy
FROM osrf/ros:jazzy-desktop

WORKDIR /my_workdir

# Keep the container alive indefinitely
CMD ["tail", "-f", "/dev/null"]

