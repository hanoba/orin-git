#!/bin/bash

# File: ekarren_wsl2_launch.sh

MY_PID=$$

cleanup() {
    echo -e "\n🛑 Beende alle Prozesse..."
    pkill -P $MY_PID
    pkill -f "rviz2"
    pkill -f "map_server"
    pkill -f "lifecycle_manager"
    pkill -f "rqt_service_caller"
    exit 0
}

trap cleanup SIGINT SIGTERM


# --- UMGEBUNG SETZEN ---
# bereits in .bashrc source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=15
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LIBGL_ALWAYS_SOFTWARE=1
# CYCLONEDDS_URI muss in .bashrc gesetzt werden
# Zum Beispiel: export CYCLONEDDS_URI=/home/harald/orin-git/ros2/wsl2/cdds_az-kengo.xml

# Konstanten
MAP_YAML="/home/harald/orin-git/ros2/map/garten_map_10cm.yaml"
LIDAR_X=0.8

clear
echo "🚀 Starte ROS2-Ausgabesystem für E-Karren..."

cd /home/harald/orin-git/ros2
pkill -9 -f _ros2_daemon
ros2 daemon start

# Statischer Transform: Lidar zu base_link
ros2 run tf2_ros static_transform_publisher $LIDAR_X 0 0 0 0 0 base_link lidar &

# Odom ist jetzt fest mit dem Roboter verbunden (da keine Encoder vorhanden)
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link &

# RViz
rviz2 -d config/ekarren_wsl2.rviz &

# Map Server (Damit wir die Karte im Hintergrund sehen)
ros2 run nav2_map_server map_server \
    --ros-args \
    -p yaml_filename:=$MAP_YAML &

# Lifecycle Manager (Nur für den Map Server)
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p node_names:="['map_server']" \
  -p autostart:=true &

# GUI zur Service-Auswahl
ros2 run rqt_service_caller rqt_service_caller &

wait
