#!/bin/bash

MY_PID=$$

cleanup() {
    echo -e "\n🛑 Beende NavigatorNode"
    pkill -P $MY_PID
    pkill -f "python3 NavigatorNode.py"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Konstanten
LIDAR_X=0.8

clear
echo "🚀 Starte System..."

cd /root/ros2
ros2 daemon start

# --- NAVIGATION ---
echo "🚀 Starte Navigator..."
python3 NavigatorNode.py

# Statischer Transform: Lidar zu base_link
ros2 run tf2_ros static_transform_publisher $LIDAR_X 0 0 0 0 0 base_link lidar &

# Transform 2 (NEU): Map -> Odom (Ersetzt AMCL)
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom &

echo "✅ System läuft mit Ground Truth von der Bridge."
