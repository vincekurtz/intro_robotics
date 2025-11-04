# Introduction to Robotics: CSE 375/475

## Helpful links

Turtlebot getting started: https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/

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
