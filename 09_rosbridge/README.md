# `webbot` -- Web Teleop of Turtlebot

This document describes how to remotely teleop a simulated turtlebot via a Web interface.

--- 

## Installation
These instructions assume the use of ROS Noetic on Ubuntu 20.04.  We'll need to install some software that wasn't initially installed on our IE 482/582 Ubuntu image.

- Update `apt`:
    ```
    sudo apt-get update
    ```

    
- Install Cesium.  This is overkill, and we won't really use Cesium for this application.  However, Cesium ships with a nice utility for running a node.js web server.  Cesium publishes new updates monthly; Version 1.83 is from July, 2021.  See https://cesium.com for more info.
    ```
    mkdir ~/cesium
    cd ~/cesium
    wget https://github.com/CesiumGS/cesium/releases/download/1.83/Cesium-1.83.zip
    unzip Cesium-1.74.zip
    rm Cesium-1.74.zip
    ```
    
- Install `node.js`:
    ```
    cd ~/cesium
    curl -sL https://deb.nodesource.com/setup_10.x | sudo -E bash -
    sudo apt-get install -y nodejs
    npm install    
    ```

- Install `roslibjs`:
    ```
    cd ~/Downloads
	rm -rf spring2023
    git clone https://github.com/IE-482-582/spring2023.git
    cd spring2023/09_rosbridge
    cp -a roslib/. ~/cesium/roslib
    ```
    
- Copy Web page into Cesium space
    ```
    cd ~/Downloads/spring2023/09_rosbridge
    cp -a webbot/. ~/cesium/webbot
    ```
        
--- 

## Running the System

### Terminal 1 - Start rosbridge server:
```
cd ~
roslaunch rosbridge_server rosbridge_websocket.launch 
```

### Terminal 2 - Start the turtlebot sim in Gazebo:
```
export TURTLEBOT3_MODEL=burger 
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

### Terminal 3 - Start the Web Server:
- First, find the IP address of your computer:
    ```
    ifconfig
    ```
  
- Next, start the web server:    
    ```
    cd ~/cesium
    node server.cjs --public
    ```
    The `--public` flag is optional.  It will allow you to view/control the turtlebot from any browser on your network (e.g., from your smartphone).  


### Edit Web page to connect to rosbridge
- Take a look at the output of `ifconfig` (in Terminal 3).  There should be field named `inet addr`.  Copy the "dotted quad", which will be 4 numbers of the form `xxx.xxx.xxx.xxx`.
- Open `~/cesium/webbot/index.html` in a text editor.  Edit the line that reads
    ```
    var ROS_IP = 'ws://192.304.5.284:9090';
    ```
    with your IP address.  Be sure to leave the `:9090` port.

### Open the `index.html` page in your Web browser:
visit [localhost:8080/webbot](localhost:8080/webbot) or replace `localhost` with the IP address of your computer.

