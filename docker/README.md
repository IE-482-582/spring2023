

# Installation

## Windows

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
 
 
## Mac

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


FIXME -- Is docker daemon running?
Just open the docker desktop app

---
 			
 			
 start docker
 	
 docker run -it -e DISPLAY=192.168.32.28:0 --name ubuntu ubuntu
 
 
 
 
 
---

# Building the course Docker image

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
    # cd [place where Dockerfile exists]
    # docker build --platform=linux/amd64 -t ub_ros_image_v2023-02-02 .     # the period at the end is not a typo
    ```
    ```
    docker build --platform=linux/amd64 -t ub_ros_image_v2023-02-02 https://raw.githubusercontent.com/IE-482-582/spring2023/main/docker/v2023-02-02/Dockerfile
    ```
    
    - See https://docs.docker.com/build/building/multi-platform/


## Create a named **container** from our image.

We can create multiple containers from the same image.  For now, we just need to create one container, named `ub_ros_container_v2023-02-02`.

### Windows Host
docker create -t \
	--privileged \
	--platform=linux/amd64 \
	--env="DISPLAY=host.docker.internal:0.0" \
	--net=host \
	--name ub_ros_container_v2023-02-02 ub_ros_image_v2023-02-02 \

    - FIXME -- Need volume info/bridge
    
### Mac Host
docker create -t \
	--privileged \
	--platform=linux/amd64 \
	--env=“DISPLAY” \
	-v /tmp/.X11-unix:/tmp/.X11-unix:rw  \
	--net=host \
	--name ub_ros_container_v2023-02-02 ub_ros_image_v2023-02-02 


#docker create -t \
#	--privileged \
#	--platform=linux/amd64 \
#	--env="DISPLAY=host.docker.internal:0.0" \
#	--net=host \
#	--name ub_ros_container_v2023-02-02 ub_ros_image_v2023-02-02 \



### Linux Host
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
	--name ub_ros_container_v2023-02-02 ub_ros_image_v2023-02-02 

# export containerId=$(docker ps -l -q)
#    osrf/ros:noetic-desktop-full \
#    # rosrun turtlesim turtlesim_node
#    roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch



docker create -t \
	--privileged \
	--platform=linux/amd64 \
	--env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"  \
	--net=host \
	--name ub_ros_container_v2023-02-02 ub_ros_image_v2023-02-02 

docker create -t \
    --privileged \
    --platform=linux/amd64 \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --net=host \
    # --name ub_ros_container_v2023-02-02 ub_ros_image_v2023-02-02 \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    osrf/ros:noetic-desktop-full \
    rqt
export containerId=$(docker ps -l -q)

docker run -it \
    --privileged \
    --platform=linux/amd64 \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    osrf/ros:noetic-desktop-full \
    rqt

```
docker create -t --privileged --platform=linux/amd64 -v ~/Downloads/tmpdocker:/root/share --net=host --name ub_ros_container_v2023-02-02 ub_ros_image_v2023-02-02
```

FIXME -- Do we need platform in BUILD or CREATE (or both)?

docker create -t --privileged --net=host -v /home/root/catkin_ws/src:/root/catkin_ws/src --name voxl-soar rosnoetic-mavsdk-python

- Double check that the container exists:
    ```
    docker ps -a
    ```
    
Run our **container**:

# LINUX:
```
# docker exec -it --privileged -e DISPLAY=host.docker.internal:0.0 ub_ros_container_v2023-02-02 bash
docker exec -it --privileged ub_ros_container_v2023-02-02 bash
```


```
# docker run -it ub_ros_image_v2023-02-2 bash
```
This will give us an interactive terminal window running the container.
(no, this actually creates a container)


https://github.com/noshluk2/ros1_wiki/blob/main/docker/commands.md#windows :
docker run -it --name r2_pathplanning_container -e DISPLAY=host.docker.internal:0.0 -e haiderabbasi333/ros2-pathplanning-course:1 bash

-e DISPLAY=192.168.1.1:0



http://wiki.ros.org/docker/Tutorials/GUI

https://docs.docker.com/engine/reference/commandline/run/


https://stackoverflow.com/questions/41485217/mount-current-directory-as-a-volume-in-docker-on-windows-10





docker create -t --privileged --net=host -v /home/root/catkin_ws/src:/root/catkin_ws/src --name voxl-soar rosnoetic-mavsdk-python


When you're done with your interactive session, simply type `exit` in the terminal window.




---

# Useful Docker Commands

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
 
 https://github.com/noshluk2/ros1_wiki/blob/main/docker/commands.md
