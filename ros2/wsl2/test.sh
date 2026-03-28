#!/bin/bash
export ROS_DOMAIN_ID=15
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/harald/orin-git/ros2/wsl2/cyclonedds_wsl2.xml

#ros2 topic list
ros2 run demo_nodes_cpp listener
