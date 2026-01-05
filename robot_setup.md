Steps for setting up a new turtlebot:

- Install Ubuntu 24.04 using `rpi-imager`
- Check "use custom settings" to:
    - Set up local network defaults
    - Enable SSH
    - Username and password as `ubuntu`
    - Hostname as 'pi0[N]' for N = 1, 2, 3, ...
- Put the SD card into the pi, boot up
- Find the new IP with `nmap`
- SSH into the pi
- (optional, if low RAM) Add some swap space, per 
  [these instructions](https://www.digitalocean.com/community/tutorials/how-to-add-swap-space-on-ubuntu-22-04)
- Follow remaining instructions from 
  [Turtlebot quickstart guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup)
    - Install ROS2 jazzy (`ros-jazzy-ros-base`)
    - Install and build turtlebot packages (may need to use 1 thread, swap space)
    - Set up `.bashrc`
- Set the `ROS_DOMAIN_ID` in `.bashrc` to match `N` from above.
- Set the `LDS_MODEL` and `TURTLEBOT3_MODEL`
- Follow the openCR firmware update stuff from the tutorials.

