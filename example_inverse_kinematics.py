#!/usr/bin/env python

import time
import numpy as np
import rtde_control

##
#
# A simple exmaple of solving an inverse kinematics problem for the UR3e arm. 
#
##

# Set up the controller
robot_ip = "192.168.1.100"
rtde_c = rtde_control.RTDEControlInterface(robot_ip)

# Define a desired end-effector pose
x = np.array([-0.3, -0.2, 0.2, 0.0, np.pi, 0.0])

# Solve IK for corresponding joint angles
q = rtde_c.getInverseKinematics(x)
print("IK solution:", q)

# Shut down the controller
rtde_c.stopScript()

