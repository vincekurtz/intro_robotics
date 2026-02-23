#!/usr/bin/env python

import time
import numpy as np
import rtde_receive

##
#
# A simple example of reading proprioceptive data from the robot arm.
#
##

# Create a receiver object
robot_ip = "192.168.1.100"
rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)

# Get the current joint angles.
q = rtde_r.getActualQ()
print("Current q:", q)

# Get the current end-effector pose.
x = rtde_r.getActualTCPPose()
print("End-effector pose:", x)

# Cleanup
rtde_r.stopScript()

