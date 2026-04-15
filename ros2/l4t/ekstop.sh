#!/bin/bash
# File: ekstop.sh
# Stoppt den ROS2-Docker-Container und den Lidarsensor

cd /home/harald/orin-git/ros2/l4t/
sudo docker restart ros2_dev_container
python3 /home/harald/orin-git/ydlidar/StopLidar.py
