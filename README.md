# Introduction to Robotics: CSE 375/475

This repository contains source code for the DePaul University course taught in
Winter 2026. 

We'll use Docker to handle dependencies and setup (Ubuntu 24.04 LTS, ROS2
Jazzy). Note that for ROS2 networking to work properly, the core docker engine
must be used: **Docker Desktop does not allow for a network configuration
compatible with ROS2.** As a result, the host machine must run Linux---either
natively or via virtual machine with bridged networking.

## Basics

Prerequisites: 
- Install [Docker Engine](https://docs.docker.com/engine/install/) (not Docker
  Desktop) on the remote PC.
- Clone this repository: `git clone
  https://github.com/vincekurtz/intro_robotics.git`.
- Enter the cloned repo on the remote PC: `cd intro_robotics`.
- Connect both the remote PC and the turtlebot to the local network
  (`Buffalo-G-896C`).

### Remote PC (Docker)

Start the container (`-d` runs in detached mode, `--build` builds the image
first, if needed):
```
docker compose up --build -d
```
Building the container may take a few minutes the first time: after that it will
be much faster.

Join the container, launching a new terminal:
```
docker compose exec robotics_env bash
```

To check if there are running containers:
```
docker ps
```

To stop the container:
```
docker compose down
```

### Turtlebot

Identify the IP address of the turtlebot. 
- Option 1: Connect a keyboard and monitor to the Raspberry Pi. Log in and check manually
  with `ip addr`. 
- Option 2: list all devices on the local network with `nmap -sn
  192.168.11.0/24` (replace `192.168.11` with the appropriate local subnet
  prefix).

SSH into the turtlebot3 (password is username):
```
ssh ubuntu@[IP address]
```

### Communication Test

To verify that ROS2 messages are passed properly between the robot and the
remote PC:

On the remote PC:
```
ros2 run demo_nodes_cpp talker
```

On the turtlebot:
```
ros2 topic list
```

You should see a `chatter` topic with messages that match what the remote PC is
sending.

### Keyboard Teleoperation

On the turtelebot:
```
ros2 launch turtlebot3_bringup robot.launch.py
```

On the remote PC:
```
ros2 run turtlebot3_teleop teleop_keyboard
```

When you're done, stop any running processes with `[CTRL-C]`.

### Custom Scripts

The docker setup includes a bind mount from `course_pkg` on the host to
`~/ros2_ws/src/course_pkg` in the container. That means you can edit files on
the host and see the changes reflected immediately in the container. We'll use
this to design and run custom ROS2 scripts in the `course_pkg` package.

To use the custom package, we first need to build it. On the remote PC:
```
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Then we can run custom scripts/launch files/executables/etc. For example:
```
ros2 run course_pkg hello_world
```

Any changes to python scripts on the host should be reflected immediately, due
to the bind mount and the `--symlink-install` option above.

## Troubleshooting

- Are the robot and the remote PC both on the same local network?
- Check the ip addresses with `ip addr`. The local IPs should look something
  like `192.168.11.4`.
- If you're using a virtual machine as the host PC, make sure bridged networking
  is enabled (not NAT).
- Run the [multicast
  test](https://docs.ros.org/en/rolling/How-To-Guides/Installation-Troubleshooting.html#enable-multicast):
  `ros2 multicast receive` on the robot, then `ros2 multicast send` on the
  remote PC, and vice versa.
- Check that `ROS_DOMAIN_ID` matches between the remote PC and the robot (`echo
  $ROS_DOMAIN_ID`). If not, edit the domain ID of the remote PC by editing
  `Dockerfile`. You'll need to rebuild the container for the changes to take
  effect. 
- If the multicast test works and `ROS_DOMAIN_ID`s match, [check the firewall
  configuration](https://stackoverflow.com/questions/75006253/ros2-on-multiple-machines-ros2-multicast-working-talker-listener-not-working).

## Helpful links

- [Turtlebot getting started guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)

- [General ROS2 troubleshooting](https://docs.ros.org/en/rolling/How-To-Guides/Installation-Troubleshooting.html)

- [Docker and ROS2 blog post](https://roboticseabass.com/2023/07/09/updated-guide-docker-and-ros2/)

- [Ubuntu Virtual Machine tutorial](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview)
