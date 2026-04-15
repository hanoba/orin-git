#!/bin/bash
# File: eksim.sh
# Startet ROS2-System auf Orin-NX für Simulation des E-Karren

cd /home/harald/orin-git/ros2/l4t/
source /home/harald/orin-git/ros2/l4t/generate_orin_cdds_uri.sh
# Docker neu starten um etwaige Zombieprozesse abzuschiessen.
sudo docker restart ros2_dev_container
sudo docker exec -it \
    -e CYCLONEDDS_URI="$CYCLONEDDS_URI" \
    ros2_dev_container bash -c "source /root/ros2/l4t/start-ros.bash && /root/ros2/launch/nav_l4t_launch.sh"

