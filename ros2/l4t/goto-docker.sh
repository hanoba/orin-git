#!/bin/bash

sudo systemctl start eKarrenEmulator
source /home/harald/orin-git/ros2/l4t/generate_orin_cdds_uri.sh
sudo docker exec -it \
    -e CYCLONEDDS_URI="$CYCLONEDDS_URI" \
    ros2_dev_container bash -c "source /root/ros2/l4t/start-ros.bash && bash"

