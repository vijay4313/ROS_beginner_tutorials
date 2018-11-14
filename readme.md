# ROS tutorials - ROS tutorials with Automated Testing | ENPM 808X
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
---

## Overview

An implementation of [ROS tutorial](http://wiki.ros.org/ROS/Tutorials/) for a simple publisher/subscriber setup with:
* Service routines to change the published text and rates
* Manipulating reference frames with ROS tf
* Automated testing framework with ROStest and Google Test
* Record ros messages with rosbag

## Dependencies

* The implementation setup requires ROS kinetic, catkin, Google Test running on Ubuntu 16.04 distribution
* Follow the [tutorial](http://wiki.ros.org/kinetic/Installation/Ubuntu) to install ROS
* Follow the [link](https://catkin-tools.readthedocs.io/en/latest/installing.html) to install catkin
* Follow the [link](http://wiki.ros.org/gtest) to install Google test for ROS platform

## Building the repo via command-line
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone -b Week11_HW --recursive https://github.com/vijay4313/beginner_tutorials.git
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
source ./devel/setup.bash
rosrun beginner_tutorials talker
```
3. Run the listener (Subscriber) in a new terminal
```
cd ~catkin_ws
source ./devel/setup.bash
rosrun beginner_tutorials listener
```
Stop the program by typing Ctrl+C in each terminal

## Running Using Launch file
In a new terminal type:
```
cd ~catkin_ws
source ./devel/setup.bash
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
## Modifying the frequency of publishing and subscribing
Relaunch the package using the launch as:
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch beginner_tutorials pubsub.launch changeFreq:=<value>
```
## Running ROStests
Build the ROS package along with the ROS test
```
cd ~catkin_ws
catkin_make run_tests beginner_tutorials
source ./devel/setup.bash
```
Tests will automatically be conducted and results of the test are displayed as:
```
-- run_tests.py: execute commands
  /opt/ros/kinetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/venkatraman/Desktop/ENPM808/HW/Week8/catkin_ws/src/beginner_tutorials --package=beginner_tutorials --results-filename test_test1.xml --results-base-dir "/home/venkatraman/Desktop/ENPM808/HW/Week8/catkin_ws/build/test_results" /home/venkatraman/Desktop/ENPM808/HW/Week8/catkin_ws/src/beginner_tutorials/test/test1.launch 
... logging to /home/venkatraman/.ros/log/rostest-venkatraman-Inspiron-5447-17114.log
[ROSUNIT] Outputting test results to /home/venkatraman/Desktop/ENPM808/HW/Week8/catkin_ws/build/test_results/beginner_tutorials/rostest-test_test1.xml
[ INFO] [1542163951.132159360]: My name is Venkat
[ INFO] [1542163951.232335543]: My name is Venkat
[ INFO] [1542163951.332243747]: My name is Venkat
[ INFO] [1542163951.432335845]: My name is Venkat
[ INFO] [1542163951.532293076]: My name is Venkat
[ WARN] [1542163951.532492315]: The Publisher Text changed
[ INFO] [1542163951.632330242]: ENPM808
[ INFO] [1542163951.732209644]: ENPM808
[ INFO] [1542163951.832352704]: ENPM808
[Testcase: testtest1] ... ok

[ROSTEST]-----------------------------------------------------------------------

[beginner_tutorials.rosunit-test1/customMessageExistance][passed]
[beginner_tutorials.rosunit-test1/customMessageSuccess][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0
```

##### to manually run the tests later:
```
rostest beginner_tutorials test1.launch
```

## Recording & Playing ROS messages
* Recording from launch file
```
cd ~catkin_ws
source ./devel/setup.bash
roslaunch beginner_tutorials pubsub.launch record:=1
```

* Recording from individual demos
```
// Separate RUN talker and listener from instructions above (Run Demo)
rosbag record --duration=10 -a -O record_beginner_tutorials.bag
```

* Playing the recorded messages
```
rosbag play record_beginner_tutorials.bag
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
![RQT_CONSOLE_OUTPUT](https://github.com/vijay4313/ROS_beginner_tutorials/blob/Week11_HW/images/beginner_tutorials_rqt_console.png)

## TF frames

The talker (publisher) sends the transformation from world frame to local frame. To list all the frames in the package:
```
// Run the package using the launch file
// In a new terminal
cd ~catkin_ws
source ./devel/setup.bash
rosrun tf view_frames
```

To view the TF messages:
```
rosrun tf tf_echo /world /talker
```

