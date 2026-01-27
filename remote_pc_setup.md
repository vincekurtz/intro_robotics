# Remote PC Setup

The Linux machines in the lab are already configured for the class, with all
dependencies pre-installed.

If you wish to use your own personal laptop, these instructions show how to
obtain the same configuration. You will need to be running Ubuntu 24.04 (LTS),
either in a virtual machine or natively.

## Install Common Software

Install some standard software, including a better terminal, build tools, etc.

```
sudo apt update
sudo apt upgrade
sudo apt install terminator git build-essential curl wget htop
```

## Configure a Text Editor

You could try something fancy like VSCode:
```
sudo snap install code --classic
```

Or something more lightweight like vim:
```
sudo apt install vim
```
For vim, I recommend using a custom `.vimrc` configuration, like [this one](
https://gist.githubusercontent.com/vincekurtz/4cd1193e53df44773196a0832ada2a9e/raw/bb59bb7a5f4d29e3dc362a5c84f9e583cc75c215/.vimrc)

## Install ROS2 Jazzy

Follow the instructions
[here](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

You will need both `ros-dev-tools` and `ros-jazzy-desktop`.

## Create a ROS2 workspace

We'll also build and install a few Turtlebot packages while we're at it
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

## Install ROS2 SLAM and Navigation packages

```
sudo apt install \
    ros-jazzy-cartographer \
    ros-jazzy-cartographer-ros \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup
```

## Configure `.bashrc`

Put the following lines in `~/.bashrc` to set environment variables each time
a new terminal is opened:
```
export ROS_DOMAIN_ID=30
export TURTLEBOT3_MODEL=burger
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash
source /home/${USERNAME}/ros2_ws/install/setup.bash
```

