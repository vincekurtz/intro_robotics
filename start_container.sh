#!/bin/bash

# Build the Docker image. This should be fast after the first time.
docker build -t intro_robotics_image .

# Launch a container from the image
docker run -it intro_robotics_image