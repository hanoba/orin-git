#!/bin/bash

# rviz.sh

export ROS_DOMAIN_ID=15
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/home/harald/orin-git/ros2/wsl2/cyclonedds_wsl2.xml
ros2 daemon start
rviz2 -d ~/orin-git/ros2/config/ekarren_test.rviz &
