#!/usr/bin/env python

from robotiq_gripper_control import RobotiqGripper
import rtde_control

##
#
# A simple example of programatically controlling the Robotiq gripper attached
# to a UR3e arm.
#
##

# Configure a general robot controller
robot_ip = "192.168.1.100"
rtde_c = rtde_control.RTDEControlInterface(robot_ip)

# Create a gripper controller object
gripper = RobotiqGripper(rtde_c)

# Activate the gripper and initialize force and speed
print("Activating")
gripper.activate()
gripper.set_force(50)  # from 0 to 100 %
gripper.set_speed(100)  # from 0 to 100 %
    
# Perform some gripper actions
print("Moving the gripper")
gripper.open()
gripper.close()
gripper.open()
gripper.move(10)  # mm
gripper.open()

# Stop the controller
rtde_c.stopScript()

