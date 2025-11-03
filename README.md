# Introduction to Robotics: CSE 375/475

## Docker basics:

Build the image (first time only, same dir as Dockerfile).
```
docker build -t my_image
```

Run the container in detached mode (`-d`). Note that the `CMD` in the
Dockerfile must run forever and not quit. The `-it` flags denote an interactive
terminal.
```
docker run -d -it --name my_container my_image
```

Check that the container is running:
```
docker ps
```

Open new terminals to access the container:
```
docker exec -it my_container /bin/bash
```

Close the container
```
docker stop my_container
```

