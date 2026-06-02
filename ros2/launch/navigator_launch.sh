#!/bin/bash

# navigator_launch.sh

MY_PID=$$

cleanup() {
    echo -e "\n🛑 Beende alle Prozesse (Ground Truth Mode)..."
    pkill -P $MY_PID
    pkill -f "python3 ros_sim_node.py"
    pkill -f "python3 viz.py"
    pkill -f "python3 NavigatorNode.py"
    pkill -f "rqt_service_caller"
    exit 0
}

trap cleanup SIGINT SIGTERM


# --- 1. NETZWERK-FIX ---
export ROS_DOMAIN_ID=15
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/home/harald/orin-git/ros2/wsl2/cyclonedds_wsl2_local.xml
export EKARREN_VIZ_IP1="192.168.178.104"    # IP Laptop Wohnung
export EKARREN_VIZ_IP2="127.0.0.1"          # IP Desktop Wohnung

#unset CYCLONEDDS_URI CYCLONE_DDS_URI
#export ROS_LOCALHOST_ONLY=1

# Konstanten
#MAP_YAML="/home/harald/orin-git/ros2/map/garten_map_10cm.yaml"
LIDAR_X=0.8
SIM_TIME=false

clear
echo "🚀 Starte Host-Simulation des E-Karren..."

cd /home/harald/orin-git/ros2
pkill -9 -f _ros2_daemon
ros2 daemon start

# --- 2. CORE KOMPONENTEN ---

# PyGame-Simulator
python3 ros_sim_node.py --ros-args \
    --log-level error \
    -p publish_odom_tf:=false &

# Statischer Transform: Lidar zu base_link
#ros2 run tf2_ros static_transform_publisher $LIDAR_X 0 0 0 0 0 base_link lidar --ros-args -p use_sim_time:=$SIM_TIME &

# Transform 2 (NEU): Map -> Odom (Ersetzt AMCL)
#ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom --ros-args -p use_sim_time:=$SIM_TIME &

# RViz
#rviz2 -d config/ransac.rviz --ros-args -p use_sim_time:=$SIM_TIME &
python3 viz.py &

# Map Server (Damit wir die Karte im Hintergrund sehen)
#ros2 run nav2_map_server map_server \
#    --ros-args \
#    -p yaml_filename:=$MAP_YAML \
#    -p use_sim_time:=$SIM_TIME &

# Lifecycle Manager (Nur für den Map Server)
#ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
#  -p node_names:="['map_server']" \
#  -p autostart:=true \
#  -p use_sim_time:=$SIM_TIME &

#echo "⏳ Warte auf Initialisierung..."
#sleep 3.0

# --- 3. FEATURE EXTRACTION & NAVIGATION ---
echo "🚀 Starte Navigator..."
python3 NavigatorNode.py \
    --ros-args \
    -p publish_odom_tf:=true \
    -p use_sim_time:=$SIM_TIME  &

# GUI zur Service-Auswahl
ros2 run rqt_service_caller rqt_service_caller --ros-args -p use_sim_time:=$SIM_TIME &


echo "✅ Host-Simulation läuft."
wait
