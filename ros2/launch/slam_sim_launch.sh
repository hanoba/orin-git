#!/bin/bash

# Wir speichern die aktuelle Prozess-ID des Skripts
MY_PID=$$

cleanup() {
    echo -e "\n🛑 Beende alle ROS-Prozesse..."
    # Wir killen alle Prozesse, die MY_PID als Vater (PPID) haben
    pkill -P $MY_PID
    # Sicherheitshalber alle Python-Nodes dieses Projekts
    pkill -f "python3 bridge.py"
    pkill -f "WallFollowerNode.py"
    pkill -f "static_transform_publisher"
    pkill -f "rviz2"
    exit 0
}

# Trap auf SIGINT (Strg+C)
trap cleanup SIGINT SIGTERM

echo "🚀 Starte Nodes..."

cd /home/harald/orin-git/ros2
ros2 run tf2_ros static_transform_publisher 0.4 0 0 0 0 0 base_link lidar --ros-args -p use_sim_time:=true &
rviz2 -d config/Map.rviz --ros-args -p use_sim_time:=true &
python3 WallFollowerNode.py  --ros-args -p use_sim_time:=true &
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=config/slam_params.yaml &
python3 bridge.py 
