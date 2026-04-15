#!/bin/bash
# File: ekemu.sh
# Startet ROS2-System auf Orin-NX für die Emulation des E-Karrens mit ROSMASTER

cd /home/harald/orin-git/ros2/l4t/
sudo systemctl start eKarrenEmulator
source /home/harald/orin-git/ros2/l4t/generate_orin_cdds_uri.sh
# Docker neu starten um etwaige Zombieprozesse abzuschiessen.
sudo docker restart ros2_dev_container
sudo docker exec \
    -e CYCLONEDDS_URI="$CYCLONEDDS_URI" \
    ros2_dev_container bash -c "source /root/ros2/l4t/start-ros.bash && /root/ros2/launch/emulator_l4t_launch.sh"

