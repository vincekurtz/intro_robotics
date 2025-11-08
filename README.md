# Introduction to Robotics: CSE 375/475

## Helpful links

Turtlebot getting started: https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/

Blog post: https://blog.robotair.io/the-complete-beginners-guide-to-using-docker-for-ros-2-deployment-2025-edition-0f259ca8b378

General ROS2 troubleshooting: https://docs.ros.org/en/rolling/How-To-Guides/Installation-Troubleshooting.html
including multicast tests.

Fix for issue with multicast working, but topics not visiable across machines:
https://stackoverflow.com/questions/75006253/ros2-on-multiple-machines-ros2-multicast-working-talker-listener-not-working

https://roboticseabass.com/2023/07/09/updated-guide-docker-and-ros2/

Initial sanity checks:
```
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener
```

## Docker basics:

Start the container (`-d` runs in detached mode, `--build` builds the image
first, if needed):
```
docker compose up --build -d
```

Join the container, in a new terminal:
```
docker compose exec intro_robotics bash
```

Stop the container
```
docker compose down
```

To check if there are running containers:
```
docker ps
```

## Turtlebot3 Bringup

SSH into the turtlebot3 (password is username):
```
ssh ubuntu@[IP address]
```

Start and enter the docker container on the host machine (see above).

On the turtelebot, 
```
ros2 launch turtlebot3_bringup robot.launch.py
```

On the host machine (in the docker container):
```
ros2 topic list
```

Remote control from the host machine (in the docker container):
```
ros2 run turtlebot3_teleop teleop_keyboard
```