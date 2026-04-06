#!/bin/bash

MY_PID=$$

cleanup() {
    echo -e "\n🛑 Beende alle Prozesse (Ground Truth Mode)..."
    pkill -P $MY_PID
    pkill -f "rviz2"
    pkill -f "map_server"
    pkill -f "lifecycle_manager"
    exit 0
}

trap cleanup SIGINT SIGTERM


# --- 1. NETZWERK-FIX ---
export ROS_DOMAIN_ID=15
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/home/harald/orin-git/ros2/wsl2/cyclonedds_wsl2.xml
export LIBGL_ALWAYS_SOFTWARE=1

# Pfade
MAP_YAML="/home/harald/orin-git/ros2/map/garten_map_10cm.yaml"
LIDAR_X=0.8

clear
echo "🚀 Starte System im GROUND TRUTH Modus..."

cd /home/harald/orin-git/ros2
ros2 daemon start

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

wait
