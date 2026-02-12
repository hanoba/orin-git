#!/bin/bash

export ROS_DOMAIN_ID=15
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///root/ros2/l4t/cyclonedds_l4t.xml
ros2 run demo_nodes_cpp talker
