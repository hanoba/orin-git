#!/bin/bash
# File: ekarren.sh
# Startet ROS2-System auf Orin-NX für den E-Karren (echte Hardware)

cd /home/harald/orin-git/ros2/l4t/
source /home/harald/orin-git/ros2/l4t/generate_orin_cdds_uri.sh
# Docker neu starten um etwaige Zombieprozesse abzuschiessen.
sudo docker restart ros2_dev_container
echo "./ekarren_l4t_launch.sh"
sudo docker exec -it \
    -e CYCLONEDDS_URI="$CYCLONEDDS_URI" \
    ros2_dev_container bash -c "source /root/ros2/l4t/start-ros.bash && bash"

