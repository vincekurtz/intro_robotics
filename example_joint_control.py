#!/usr/bin/env python

import time
import numpy as np
import rtde_control

##
#
# Move the robot's joints programatically.
#
##

# Set up the controller
robot_ip = "192.168.1.100"
rtde_c = rtde_control.RTDEControlInterface(robot_ip)

# Set some example joint angles
q1 = np.array([0.0, -1.56, 0, -1.47, 0, 0])
q2 = np.array([0.2, -1.56, 0, -1.47, 0, 0])

# Move to the given joint configurations
print("Moving to q1")
rtde_c.moveJ(q1)

print("Moving to q2")
rtde_c.moveJ(q2)

# Shut down the controller
rtde_c.stopScript()

