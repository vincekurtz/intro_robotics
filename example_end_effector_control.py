#!/usr/bin/env python

import time
import numpy as np
import rtde_control

##
#
# Move the robot's end-effector programatically.
#
##

# Set up the controller
robot_ip = "192.168.1.100"
rtde_c = rtde_control.RTDEControlInterface(robot_ip)

# Set some example end-effector poses.
x1 = np.array([0.0, 0.0, 1.2, 0.0, 0.0, 0.0])
x2 = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])

# Move to the given poses
print("Moving to x1")
rtde_c.moveL(x1, speed=0.1)

print("Moving to x2")
rtde_c.moveL(x2, speed=0.1)

# Shut down the controller
rtde_c.stopScript()

