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

# Go home
q1 = np.array([0.0, -1.56, 0, -1.47, 0, 0])
rtde_c.moveJ(q1)

# Set some example end-effector poses.
x1 = np.array([-0.0164, -0.3697, 0.61,
               0.1268, 2.1744, -2.1586])
x2 = np.array([-0.0164, -0.3697, 0.68,
               0.1268, 2.1744, -2.1586])

# Move to the given poses
print("Moving to x1")
rtde_c.moveL(x1, speed=0.1)

print("Moving to x2")
rtde_c.moveL(x2, speed=0.1)

# Shut down the controller
rtde_c.stopScript()

