# Walker_robot
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
---

## Overview
This program simulates a turtlebot object avoidance behaviour. It does so by simply moving forward until it senses an obstacle within a particular range. Whenever there is any obstacle alonng the path, it rotates around its center until the path is clear and then again starts moving forward. The program also has the capability to enable and disable rosbag recording, which when enabled can record all the topics except '/camera' as well as there is an option ot apecify recording time. The rosbag file is generated in the results directory and sabed as "walker.bag". This rosbag file can then be played to see all the topic messages which had been published.

## Dependencies
This program works on a device running Ubuntu 16.04 and ROS Kinetic Kame. It will also need Gazebo 7.x (part of ros-kinetic-desktop-full package) and Turtlebot simulation stack. If full destop option was selecting while installing ROS kinetic, it is not required to install Gazebo separately.

* To install ROS Kinetic Kame in Ubuntu 16.04, follow the steps in this [link](http://wiki.ros.org/kinetic/Installation/Ubuntu).

* To install catkin, follow the installation steps in this [link](http://wiki.ros.org/catkin).

* To install turtlebot simulation stack. In a terminal:

```
sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
source /opt/ros/kinetic/setup.bash
```
## Build Instructions

To run this code in a catkin workspace:
```
cd ~/catkin_ws/
source devel/setup.bash
cd src/
git clone --recursive https://github.com/sbrahma0/walker_robot.git
cd ..
catkin_make
```
If you do not have a catkin workspace:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/sbrahma0/walker_robot.git
cd ..
catkin_make
```
## Run Instructions using Launch File
After completing the building step, go to the workspace in the terminal and give the following commands.
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch walker_robot walker_robot.launch 
```
The above command will automatically start the Gazebo application and the turtlebot will start moving while avoiding obstacles.

To enable the rosbag file recording for a specific time please give the following commands in a new terminal. If there are any previous launch files running , please do exit from them before starting another launch file.
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch walker_robot walker_robot.launch record:=true record_time:=25
```
If the record time is not specified, by default it will be recorded for 30 sec. The topics from Camera are not recorded since they tend to make bag files very big.

The bag file is stores in results folder as walker.bag

## Inspecting the Bag file generated
To get more information about the generated rosbag file, open the results directory in terminal and run the following command:
```
rosbag info walker.bag
```

## Playing the rosbag file
The rosbag file records all the messages being published on to the terminal. For example, the bag file generated and saved in the results folder.

Open the results folder in a new terminal and run:
```
rosbag play walker.bag
```
You will see that all the messages recorded are being replayed. 
To verify this, run
```
rostopic list
```
We will see that all the topics of the turtlebot are active.
To observe the change in velocity when rotating, run
```
rostopic echo /mobile_base/commands/velocity 
```
The change in velicoty in z angular indicates the turtlebot rotating.








