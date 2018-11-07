# ROS tutorials - ROS services & RQT Console | ENPM 808X
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
---

## Overview

An implementation of [ROS tutorial](http://wiki.ros.org/ROS/Tutorials/) for a simple publisher/subscriber setup with service routines to change the published text and rates

## Dependencies

* The implementation setup requires ROS kinetic and catkin running on Ubuntu 16.04 distribution
* Follow the [tutorial](http://wiki.ros.org/kinetic/Installation/Ubuntu) to install ROS
* Follow the [link](https://catkin-tools.readthedocs.io/en/latest/installing.html) to install catkin

## Building the repo via command-line
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone -b Week10_HW --recursive https://github.com/vijay4313/ROS_beginner_tutorials.git
cd ..
catkin_make
```

## Running Demo
To run the demo:
1. Open a new terminal and type 
```
roscore
```
This initiates the ROS system and its dependencies

2. Run the talker (Publisher) in a new terminal
```
cd ~catkin_ws
source .devel/setup.bash
rosrun beginner_tutorials talker
```
3. Run the listener (Subscriber) in a new terminal
```
cd ~catkin_ws
source .devel/setup.bash
rosrun beginner_tutorials listener
```
Stop the program by typing Ctrl+C in each terminal

## Running Using Launch file
```
roslaunch beginner_tutorials pubsub.launch
```

The launcher will launch both publisher and subscriber in separate terminals with default text and publish frequency

## Modifying the display text
In a separate terminal:
```
cd ~/catkin_ws/
source devel/setup.bash
rosservice call  /talker/change_string  'ENPM808X'
```

## Setting logger levels
```
rosrun rqt_logger_level rqt_logger_level
```
Select the node and select logger level as needed form the GUI

## Viewing the logger messages
In a new terminal
```
rqt_console
```
![RQT_CONSOLE_OUTPUT](https://github.com/vijay4313/ROS_beginner_tutorials/blob/Week10_HW/images/beginner_tutorials_rqt_console.png)

