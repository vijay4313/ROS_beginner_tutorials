# ROS tutorials - Publisher/Subscriber | ENPM 808X
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
---

## Overview

An implementation of [ROS tutorial](http://wiki.ros.org/ROS/Tutorials/) for a simple publisher/subscriber setup

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
git clone https://github.com/vijay4313/ROS_beginner_tutorials.git
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
