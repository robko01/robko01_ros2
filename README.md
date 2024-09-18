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

 - Run the controller
```
 ros2 run robko01_ros2 service -- /dev/ttyUSB0 orlin369
```
