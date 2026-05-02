#!/bin/bash
# File: eksim.sh
# Startet ROS2-System auf Orin-NX für Simulation des E-Karren

cd /home/harald/orin-git/ros2/l4t/
source /home/harald/orin-git/ros2/l4t/generate_orin_cdds_uri.sh
# Docker neu starten um etwaige Zombieprozesse abzuschiessen.
echo "Kommando um ROS2-System auf Orin-NX für Simulation des E-Karren zu starten: ./nav_l4t_launch.sh"
sudo docker exec -it \
    -e CYCLONEDDS_URI="$CYCLONEDDS_URI" \
    ros2_dev_container bash -c "source /root/ros2/l4t/start-ros.bash && bash"


