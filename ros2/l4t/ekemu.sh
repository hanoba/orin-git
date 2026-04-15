#!/bin/bash
# File: emulator.sh
# Startet ROS2-System auf Orin-NX für die Emulation des E-Karrens mit ROSMASTER

sudo systemctl start eKarrenEmulator
source /home/harald/orin-git/ros2/l4t/generate_orin_cdds_uri.sh
sudo docker exec \
    -e CYCLONEDDS_URI="$CYCLONEDDS_URI" \
    ros2_dev_container bash -c "source /root/ros2/l4t/start-ros.bash && /root/ros2/launch/emulator_l4t_launch.sh"

