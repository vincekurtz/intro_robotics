# Introduction to Robotics: CSE 375/475

This repository contains source code for the DePaul University course taught in
Winter 2026. 

> [!NOTE]
> For the previous (docker-based) version, see [this branch](TODO).

## Basics

For the remote PC used to send commands to robots, we use Ubuntu 24.04 and
ROS2 ``Jazzy Jalisco.'' The computers in the lab are already configured with
this setup. If you wish to use your own personal computer (either via virtual
machine or with Ubuntu 24.04 installed manually), see [these
instructions](remote_pc_setup.md).

### Remote PC Setup

Clone this repository to `~/ros2_ws/src/`:
```
cd ~/ros2_ws/src/
git clone https://github.com/vincekurtz/intro_robotics.git
```

Make sure the remote PC is connected to the class router (`Buffalo-G-896C`).

### Turtlebot Setup

Identify the IP address of the turtlebot (it should automatically connect to the
class router).
- Option 1: Connect a keyboard and monitor to the Raspberry Pi. Log in and check manually
  with `ip addr`. 
- Option 2: list all devices on the local network with `nmap -sn
  192.168.11.0/24` (replace `192.168.11` with the appropriate local subnet
  prefix).

SSH into the turtlebot3 (the password is the username):
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

### Custom Nodes

To run your own ROS2 nodes, you'll need to first compile the package.
```
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Then we can run custom scripts. For example,

```
ros2 run intro_robotics hello_world
```

Once the package is compiled, you should be able to change any existing python
scripts without recompiling, thanks to the `--symlink-install` option above.

## Troubleshooting

The remote PC can't seem to communicate with the robot! What should I do?

- Are the robot and the remote PC both on the same local network?
- Check both ip addresses with `ip addr`. The local IPs should look something
  like `192.168.11.4`.
- If you're using a virtual machine as the remote PC, make sure bridged networking
  is enabled (not NAT).
- Run the [multicast
  test](https://docs.ros.org/en/rolling/How-To-Guides/Installation-Troubleshooting.html#enable-multicast):
  `ros2 multicast receive` on the robot, then `ros2 multicast send` on the
  remote PC, and vice versa.
- Check that `ROS_DOMAIN_ID` matches between the remote PC and the robot (`echo
  $ROS_DOMAIN_ID`). If not, edit the domain ID of the remote PC by editing
  `~/.bashrc`. The change will take effect once you open a new terminal window.
- If the multicast test works and `ROS_DOMAIN_ID`s match, [check the firewall
  configuration](https://stackoverflow.com/questions/75006253/ros2-on-multiple-machines-ros2-multicast-working-talker-listener-not-working).
- If the multicast test works and `ROS_DOMAIN_ID`s match, but there are still
  issues seeing topics on both machines, try restarting the ros2 daemon on one
  or both machines: `ros2 daemon stop`, then `ros2 daemon start`. That's
  particularly helpful if the network configuration has changed.

## Helpful links

- [Turtlebot getting started guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)

- [General ROS2 troubleshooting](https://docs.ros.org/en/rolling/How-To-Guides/Installation-Troubleshooting.html)

- [Ubuntu Virtual Machine tutorial](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview)

