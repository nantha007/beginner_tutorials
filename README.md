# ROS Tutorials

## Overview

The program is about the ROS Subscriber and Publisher. A publisher node called "talker" is created and it send message "Ros tutorial" and a number on topic chatter. A subcriber node is called "listener" is created and it receives message on topic chatter.

## System requirement

The following settings are required to run this program.
* Ubuntu Xenial
* ROS Kinetic

## Instructions for creating catkin workspace

To create ctakin workspace, open terminal and type the following commands
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

## Instructions to setup catkin workspace

Once catkin workspace is created, open terminal and type the following commands to clone the repository.

``` 
cd ~/catkin_ws/src/
git clone --recursive https://github.com/nantha007/beginner_tutorials.git
```

Once repository is cloned, type the following commands to setup the repository.
```
cd ~/catkin_ws/
source devel/setup.bash
catkin_make
```

## Instructions to Run the program 

Open a terminal and type following commands to run the master node.
```
source devel/setup.bash
roscore
```

Open another instance of the terminal and type the following commands to run publisher Node talker.
```
source devel/setup.bash
rosrun beginner_tutorials talker
```

Open another instance of the terminal and type the following commands to run subscriber Node listener.
```
source devel/setup.bash
rosrun beginner_tutorials listener
```

