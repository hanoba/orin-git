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


# --- 1. UMGEBUNG SETZEN ---
# bereits in .bashrc source /opt/ros/humble/setup.bash  # WICHTIG: Ersetze $ROS_DISTRO durch deine Version (z.B. humble)
export ROS_DOMAIN_ID=15
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/home/harald/orin-git/ros2/wsl2/cyclonedds_wsl2.xml
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1 # Zusätzlicher Fix für WSL-Grafikfehler

# Pfade
MAP_YAML="/home/harald/orin-git/ros2/map/garten_map_10cm.yaml"

clear
echo "🚀 Starte System im GROUND TRUTH Modus..."

# --- 2. START-SEQUENCE MIT TIMING ---

echo "--- Starte ROS2 Daemon..."
ros2 daemon stop && ros2 daemon start
sleep 2

echo "--- Starte Map Server..."
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$MAP_YAML &
sleep 2

echo "--- Aktiviere Map Server via Lifecycle Manager..."
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p node_names:="['map_server']" \
  -p autostart:=true &
sleep 3 # Gib dem Map Server Zeit, die Karte wirklich zu laden

echo "--- Starte RViz2..."
# Hier nutzen wir 'ros2 run' statt den direkten Pfad, um sicherzugehen
ros2 run rviz2 rviz2 -d /home/harald/orin-git/ros2/config/ekarren_wsl2.rviz &

wait
