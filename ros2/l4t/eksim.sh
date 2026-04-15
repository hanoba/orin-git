#!/bin/bash
# File: simulator.sh
# Startet ROS2-System auf Orin-NX für Simulation des E-Karren

source /home/harald/orin-git/ros2/l4t/generate_orin_cdds_uri.sh
sudo docker exec \
    -e CYCLONEDDS_URI="$CYCLONEDDS_URI" \
    ros2_dev_container bash -c "source /root/ros2/l4t/start-ros.bash && /root/ros2/launch/nav_l4t_launch.sh"

