This document describes how to run Ubuntu 20.04 / ROS Noetic in Docker. This is aimed at the IE 482/582 course.

There are 4 key sections:
1. Installing the Docker software
2. Building the course Docker Image
3. Creating a Docker Container
4. Running the Docker Container
5. Links
6. Helpful Docker Commands

---

# 1. Installing Docker

## 1.1 Windows

These Windows instructions were adapted from the following sources:
- http://wiki.ros.org/docker/Tutorials/Docker
- https://www.youtube.com/watch?v=qWuudNxFGOQ&t=146s

1. Download and install `xlaunch` for Windows:
    - https://sourceforge.net/projects/vcxsrv/
    
2. Download and install Docker Desktop for Windows:
    - https://docs.docker.com/desktop/install/windows-install/ 
    - You will need to use the "WSL 2 backend" option
    - You can skip the tutorial if you wish.
    - When prompted, be sure to *reboot your computer*.
 
 
## 1.2 Mac

These Mac instructions were adpated from https://www.youtube.com/watch?v=cNDR6Z24KLM

1. Download and install `xquartz`:
    - https://www.xquartz.org/
    - When prompted, be sure to *reboot your computer*.

2. Download and install Docker Desktop for Mac:
    - https://docs.docker.com/desktop/install/mac-install/
    - Make sure you choose the correct Intel/Apple version.
    - You do **not** need to install `Rosetta 2`
    - You can skip the tutorial if you wish.


3. On your Mac, open a terminal and type the following:
    ```
    ifconfig en0
    ```
    
    - Copy the "dotted quad" found after `inet`.  It should be a block of four 2-3 digit numbers, like `192.168.32.28`.  You'll need this info later.
        - **Don't** grab the "dotted quad" shown after `broadcast`
 		 	
4.  On your Mac, open XQuartz (search in your apps and open). 
    - From the top menu, choose `Settings...` --> `Security`:
        - Check the box for "Allow connections from network clients"
    - In the XQuartz terminal, type `xhost` followed by the "dotted quad" you found in Step 3.:
        ```
        xhost 192.168.32.28  # (change the dotted quad as appropriate)
        ```
        
**NOTE:** I'm not sure if Steps 3 and 4 will be necessary.  We'll update this guide as necessary.

 
---

# 2. Building the course Docker image

Before continuing, I suggest you check out [the difference between a Docker "image" and a Docker "container"](https://stackoverflow.com/questions/21498832/in-docker-whats-the-difference-between-a-container-and-an-image).  Containers are like instances of an image.  Each image may be used to create multiple containers. Images are read-only. 


- We'll start by using a `Dockerfile` to create a Docker **image** named `ub_ros_image_v2023-02-02`.
- Then, we'll use the `ub_ros_image` image to create a **container** named `ub_ros_container_v2023-02-02`
- When you're doing work in class, you'll run the `ub_ros_container_v2023-02-02` container.  Any files you save in the container will be available to you next time you run that container.

1. Do we have any existing **containers**?
    ```
    docker ps -a
    ```

    - If you want to stop a running container:
        ```
        docker stop <container name>
        ```    
 
    - If you have an old container you no longer need/want, you can delete (remove) it as follows:
        ```
        docker rm <container id or container name>
        ```
        **CAUTION:** There's no "undo" command.

2. Do we have any existing **images**?
    ```
    docker images -a
    ```

    - If you want to delete an image:
        ```
        docker rmi <image id>
        ```
        **CAUTION:** There's no "undo" command.
    
3.  Let's now build our `ub_ros_image_v2023-02-02` **image**:
    ```
    docker build --platform=linux/amd64 -t ub_ros_image_v2023-02-02 https://raw.githubusercontent.com/IE-482-582/spring2023/main/docker/v2023-02-02/Dockerfile
    ```
    
    - See https://docs.docker.com/build/building/multi-platform/ for more info

---

# 3. Create a named **container** from our image.

We can create multiple containers from the same image.  For now, we just need to create one container, named `ub_ros_container_v2023-02-02`.

The procedure for creating containers is going to depend on your operating system.

## 3.1 Windows Host
```
docker create -t \
	--privileged \
	--platform=linux/amd64 \
	--env="DISPLAY=host.docker.internal:0.0" \
	--net=host \
	--name ub_ros_container_v2023-02-02 ub_ros_image_v2023-02-02 \
```
- FIXME -- Need volume info/bridge
- FIXME -- Add local linked/bridged volume


## 3.2 Mac Host
```
docker create -t \
	--privileged \
	--platform=linux/amd64 \
	--env="DISPLAY" \
	-v /tmp/.X11-unix:/tmp/.X11-unix:rw  \
	--net=host \
	--name ub_ros_container_v2023-02-02 ub_ros_image_v2023-02-02 
```
- FIXME -- Should it be `--env="DISPLAY=host.docker.internal:0.0" \`?
- FIXME -- Add local linked/bridged volume


## 3.3 Linux Host
```
#  xhost +local:root			# DANGEROUS!!!
export containerId=$(docker ps -aqf "name=ub_ros_container_v2023-02-02")
# xhost +local:`docker inspect --format='{{ .Config.Hostname }}' $containerId`
xhost +local:$containerId

XAUTH=/tmp/.docker.xauth

docker create -t \
    --privileged \
    --platform=linux/amd64 \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --net=host \
    --name ub_ros_container_v2023-02-02 ub_ros_image_v2023-02-02 
```
- FIXME -- Add local linked/bridged volume
    - `-v ~/Downloads/tmpdocker:/root/share` 
- FIXME -- Do we need platform in BUILD or CREATE (or **both**)?


- Double check that the container exists:
    ```
    docker ps -a
    ```

---


# 4. Run our **container**:

### Windows Host:
FIXME

### Mac Host:
FIXME

### Linux Host:
```
# docker exec -it --privileged -e DISPLAY=host.docker.internal:0.0 ub_ros_container_v2023-02-02 bash
docker exec -it --privileged ub_ros_container_v2023-02-02 bash
```

---

# 5. Links:

- https://github.com/noshluk2/ros1_wiki/blob/main/docker/commands.md

- https://github.com/noshluk2/ros1_wiki/blob/main/docker/commands.md#windows :
    ```
    docker run -it --name r2_pathplanning_container -e DISPLAY=host.docker.internal:0.0 -e haiderabbasi333/ros2-pathplanning-course:1 bash
    ```
- ???
    ```
    -e DISPLAY=192.168.1.1:0
    ```
- http://wiki.ros.org/docker/Tutorials/GUI
- https://docs.docker.com/engine/reference/commandline/run/
- https://stackoverflow.com/questions/41485217/mount-current-directory-as-a-volume-in-docker-on-windows-10

- When you're done with your interactive session, simply type `exit` in the terminal window.




---

# 6. Useful Docker Commands

## Containers
 
- List all containers:
```
docker ps -a
``` 
 
- Stop a running container
```
docker stop <container name> 
```

- Delete (remove) a single container
```
docker rm <container id or container name>
```


- Start a stopped/existing container
```
docker start <container id>
```

- Create an interactive container from an image 
```
docker run -it <image name>
```

**NOTE:** Each time you do this, you create a new container.  This probably isn't what you actually want to do.




## Images

- List all images
```
docker images -a
```

- Delete an image:
```
docker rmi <image id>
```
 
 
 
 --- 
 
