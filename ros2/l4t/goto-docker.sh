#!/bin/bash

sudo systemctl start eKarrenEmulator
sudo docker exec -it ros2_dev_container bash -c "source /root/ros2/l4t/start-ros.bash && bash"

