#!/bin/bash

if [ -z "$ROS_SOURCED" ]; then 
    source /opt/ros/humble/install/setup.bash
    export ROS_DOMAIN_ID=15
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI=file:///root/ros2/l4t/cyclonedds_l4t.xml
    export ROS_SOURCED=1
fi

