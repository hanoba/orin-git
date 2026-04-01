#!/bin/bash

# navigator_launch.shexport LIBGL_ALWAYS_SOFTWARE=1

export ROS_DOMAIN_ID=15
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/home/harald/orin-git/ros2/wsl2/cyclonedds_wsl2.xml
#export LIBGL_ALWAYS_SOFTWARE=1
ros2 daemon start
ros2 run teleop_twist_keyboard teleop_twist_keyboard

