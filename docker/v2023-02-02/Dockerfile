FROM osrf/ros:noetic-desktop-full

RUN apt-get update
RUN apt-get install cmake build-essential colordiff git doxygen -y
RUN apt-get install python3 python3-pip -y
RUN apt-get install git -y
RUN apt install curl 

RUN pip install --upgrade pip

# Other Python Packages
RUN pip3 install geopy

# ifconfig tool
RUN apt-get install net-tools -y
# iwconfig tool
RUN apt-get install wireless-tools -y

RUN apt-get install iputils-ping -y

# Text editor
RUN apt-get -y install nano

# ROS variables
RUN echo "" >> ${HOME}/.bashrc
RUN echo "# Set ROS Environment Variables:" >> ${HOME}/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ${HOME}/.bashrc
# RUN echo "source ${HOME}/rosbridge_ws/devel/setup.bash --extend" >> ${HOME}/.bashrc

# Catkin
# RUN echo "source ${HOME}/catkin_ws/devel/setup.bash --extend" >> ${HOME}/.bashrc
RUN mkdir -p ${HOME}/catkin_ws/src
# Need to run together in the same layer to enable catkin_make command
RUN /bin/bash -c ". /opt/ros/noetic/setup.bash; cd ~/catkin_ws; catkin_make"

RUN echo "source ~/catkin_ws/devel/setup.bash --extend" >> ${HOME}/.bashrc
# source ${HOME}/catkin_ws/devel/setup.bash

RUN /bin/bash -c "source ~/.bashrc"

RUN apt-get update && pip3 install opencv-contrib-python-headless && apt-get install ros-noetic-cv-bridge -y

RUN apt-get clean
RUN apt-get install ros-noetic-rosbridge-suite -y

# August 2022 now using psutil
RUN pip3 install psutil

# Sept 2022 need to add pandas
RUN pip3 install pandas


# Install Turtlebot3
#    - See https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/

## Install Dependent ROS 1 Packages
RUN apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy  -y
RUN apt-get install ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc  -y
RUN apt-get install ros-noetic-rgbd-launch ros-noetic-rosserial-arduino  -y
RUN apt-get install ros-noetic-rosserial-python ros-noetic-rosserial-client  -y
RUN apt-get install ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server  -y
RUN apt-get install ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro  -y
RUN apt-get install ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz  -y
RUN apt-get install ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers  -y

## Install TurtleBot3 via Debian Packages:
RUN apt install ros-noetic-dynamixel-sdk  -y
RUN apt install ros-noetic-turtlebot3-msgs  -y
RUN apt install ros-noetic-turtlebot3  -y

## Install Simulation Package
##    - See https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/
RUN /bin/bash -c "cd ~/catkin_ws/src; git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git" 
RUN /bin/bash -c ". /opt/ros/noetic/setup.bash; cd ~/catkin_ws; catkin_make"


## Test the installation:
# export TURTLEBOT3_MODEL=waffle_pi
# roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch


# https://github.com/IE-482-582/spring2023/issues/2

# FIXME -- Add Husky?
# FIXME -- Add Clover?

