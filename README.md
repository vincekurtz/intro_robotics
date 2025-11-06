# Introduction to Robotics: CSE 375/475

## Helpful links

Turtlebot getting started: https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/

Blog post: https://blog.robotair.io/the-complete-beginners-guide-to-using-docker-for-ros-2-deployment-2025-edition-0f259ca8b378

General ROS2 troubleshooting: https://docs.ros.org/en/rolling/How-To-Guides/Installation-Troubleshooting.html
including multicast tests.

Fix for issue with multicast working, but topics not visiable across machines:
https://stackoverflow.com/questions/75006253/ros2-on-multiple-machines-ros2-multicast-working-talker-listener-not-working

Initial sanity checks:
```
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener
```

## Docker basics:

Build the image (first time only, same dir as Dockerfile).
```
docker build -t intro_robotics_image .
```

Launch a container from the image:
```
docker run -it intro_robotics_image
```

Note that we can launch several containers at once with this commands. These
will be separate containers, but can communicate over ROS topics.

### Alternative version, enter several terminals in the same container
Start up the container, in detached mode (`-d`) and set up for interactive
terminals (`-it`).
```
docker run -d -it --name intro_robotics intro_robotics_image
```

Check that the container is running:
```
docker ps
```

Open new terminals to access the container:
```
docker exec -it intro_robotics /bin/bash 
```
Note that if we do this, we need to source the ROS setup file again in each
terminal:
```
source /ros_entrypoint.sh
```

Close the container
```
docker stop intro_robotics
```
