#!/usr/bin/env python

##
#
# Programatically enter freedrive mode, where
# the robot can be moved manually.
#
##

import time
import rtde_control

robot_ip = "192.168.1.100"
rtde_c = rtde_control.RTDEControlInterface(robot_ip)

try:
    # Enter free-drive mode for 5 seconds.
    start_time = time.time()
    rtde_c.freedriveMode()
    while time.time() - start_time < 5.0:
        time.sleep(0.1)
finally:
    # Stop freedrive and clean up, even if we got an error above.
    rtde_c.endFreedriveMode()
    rtde_c.stopScript()

