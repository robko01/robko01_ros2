# Intro
Robko01 ROS2 package
This document is devoted to the way ROS package software is installed, which communicates with the robot controller.

# Installation

## Environment

 - [ROS2 Iron Irwini](https://docs.ros.org/en/iron/Releases/Release-Iron-Irwini.html)

 - For better experience is good to have git client. This will will alow you to install easy from github this library. The link to the [git client](https://git-scm.com/download/win).

## Install the package

 - Go to the ROS2 workspace. In the most ways see below.
```sh
cd ~\ros2_ws\src
```

 - Clone the repo.
```sh
git clone https://github.com/robko01/robko01_ros2
```

 - Compile the library.
```sh
colcon build --packages-select robko01_ros2
```

 - Link the library to the environment.
```sh
source install/setup.bash
```

## Run the service for communication with the robot controller

 - Run the controller if interface is Serial
```sh
ros2 run robko01_ros2 service --ros-args --param port:=/dev/ttyUSB0
```

 - Run the controller if interface is TCP/IP
```sh
ros2 run robko01_ros2 service --ros-args --param host:=192.168.88.221 --param port:=10182
```

## Run the client for sending trajectory to the service

 - Run the controller
```sh
ros2 run robko01_ros2 client
```

## Install dependencies

```sh
sudo apt install ros-iron-joint-state-publisher-gui
sudo apt install ros-iron-gz-gazebo ros-iron-gz-launch ros-iron-gz-sim ros-iron-ros-gz
sudo apt install ros-iron-gazebo-ros-pkgs ros-iron-gazebo-ros-control
sudo apt install ros-iron-xacro
```

## Run the vizualization in RViz

```sh
ros2 launch robko01_ros2 display.launch.py
```

## Ensure that the robot description is loaded

```sh
ros2 param get /robot_state_publisher robot_description
```