# ENPM808X Turtlebot Walker
<a href='https://github.com/dngo13/enpm808x_walker/blob/master/LICENSE'><img src='https://img.shields.io/badge/License-BSD_2--Clause-orange.svg'/></a>
### Diane Ngo (117554960)
### ENPM 808X Fall 2021

## Overview
This is a ROS Package that uses turtlebot to simulate a roomba walker algorithm in gazebo in C++.

## Dependencies
Must have the following installed:
- Ubuntu 18.04
- ROS Melodic
- C++, roscpp
- Standard library

## Building Instructions
### If there is no catkin workspace
Create catkin workspace. Go to terminal and follow these commands:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
Then source the setup.bash file with this command:
```
source devel/setup.bash
```
Now please clone the enpm808x_walker package by doing the following:
```
cd src/
git clone https://github.com/dngo13/enpm808x_walker.git
cd ..
catkin_make
source devel/setup.bash
```
### Cloning into an existing catkin workspace
Go to the src directory for the catkin workspace. And then follow these steps
```
cd ~/catkin_ws/src/
git clone https://github.com/dngo13/enpm808x_walker.git
cd ..
catkin_make
source devel/setup.bash
```
## Running the Program
Now that the package is built, we can run the program. 
In this terminal, run ```roscore```.

### Running the launch file
Open a new terminal, source it with:
```
source devel/setup.bash
```
And then run:
```
roslaunch enpm808x_walker walker.launch
```

### View rosbag file 
There is a rosbag file already recorded in the /results directory. To view it, do the following commands in a terminal.
```
cd ~/catkin_ws/
source devel/setup.bash
cd src/enpm808x_walker/results
rosbag info rosbag_record.bag 
```

The output should be similar to this: 
```
path:        rosbag_record.bag
version:     2.0
duration:    22.3s
start:       Dec 31 1969 19:00:00.17 (0.17)
end:         Dec 31 1969 19:00:22.42 (22.42)
size:        35.2 MB
messages:    89467
compression: none [46/46 chunks]
types:       dynamic_reconfigure/Config            [958f16a05573709014982821e6822580]
             dynamic_reconfigure/ConfigDescription [757ce9d44ba8ddd801bb30bc456f946f]
             gazebo_msgs/LinkStates                [48c080191eb15c41858319b4d8a609c2]
             gazebo_msgs/ModelStates               [48c080191eb15c41858319b4d8a609c2]
             geometry_msgs/Twist                   [9f195f881246fdfa2798d1d3eebca84a]
             nav_msgs/Odometry                     [cd5e73d190d741a2f92e81eda573aca7]
             rosgraph_msgs/Clock                   [a9c97c1d230cfc112e270351a944ee47]
             rosgraph_msgs/Log                     [acffd30cd6b6de30f120938c17c593fb]
             sensor_msgs/Imu                       [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/JointState                [3066dcd76a6cfaef579bd0f34173e9fd]
             sensor_msgs/LaserScan                 [90c7ef2dc6895d81024acba2ac42f369]
             tf2_msgs/TFMessage                    [94810edda583a504dfda3829e70d7eec]
topics:      /clock                           22251 msgs    : rosgraph_msgs/Clock                  
             /cmd_vel                           222 msgs    : geometry_msgs/Twist                  
             /gazebo/link_states              22117 msgs    : gazebo_msgs/LinkStates               
             /gazebo/model_states             22119 msgs    : gazebo_msgs/ModelStates              
             /gazebo/parameter_descriptions       1 msg     : dynamic_reconfigure/ConfigDescription
             /gazebo/parameter_updates            1 msg     : dynamic_reconfigure/Config           
             /imu                             20199 msgs    : sensor_msgs/Imu                      
             /joint_states                      663 msgs    : sensor_msgs/JointState               
             /odom                              663 msgs    : nav_msgs/Odometry                    
             /rosout                            233 msgs    : rosgraph_msgs/Log                     (4 connections)
             /rosout_agg                        224 msgs    : rosgraph_msgs/Log                    
             /scan                              111 msgs    : sensor_msgs/LaserScan                
             /tf                                663 msgs    : tf2_msgs/TFMessage
```

#### Recording a New Bag File
If you want to record a new bag file then launch the launch file with the following argument like below:
```
roslaunch enpm808x_walker walker.launch record_bag:=True
```
To disable bag recording, just ignore the record_bag param as the default value is set to false in the launch file.
