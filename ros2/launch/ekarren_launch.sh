#!/bin/bash

# lidar_launch.sh

MY_PID=$$

cleanup() {
    echo -e "\n?? Beende alle Prozesse ..."
    pkill -P $MY_PID
    pkill -f "python3 eKarrenNode.py"
    exit 0
}

trap cleanup SIGINT SIGTERM

clear
echo "Starte E-Karren..."

cd /home/harald/orin-git/ros2
ros2 daemon start

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link lidar &

# E-Karren Node
python3 eKarrenNode.py 

