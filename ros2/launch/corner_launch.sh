#!/bin/bash

# corner_launch.sh

MY_PID=$$

cleanup() {
    echo -e "\n🛑 Beende alle ROS-Prozesse..."
    pkill -P $MY_PID
    pkill -f "python3 bridge.py"
    pkill -f "python3 WallFollowerNode.py"
    pkill -f "python3 SimpleCornerDetector.py" # Unser neuer Node
    pkill -f "static_transform_publisher"
    pkill -f "rviz2"
    exit 0
}

trap cleanup SIGINT SIGTERM

clear
echo "🚀 Starte Nodes (Simple Mode)..."
cd /home/harald/orin-git/ros2
ros2 daemon start

# 1. Bridge
taskset -c 16,17 python3 bridge.py &

# 2. Statischer Transform
taskset -c 18-31 ros2 run tf2_ros static_transform_publisher 0.4 0 0 0 0 0 base_link lidar --ros-args -p use_sim_time:=true &

# 3. Unser neuer Python Ecken-Detektor (ersetzt das komplexe C++ Paket)
taskset -c 4 python3 CornerDetectorNode.py --ros-args -p use_sim_time:=true &

# 4. RViz
taskset -c 18-31 rviz2 -d config/Corner.rviz --ros-args -p use_sim_time:=true &

# 5. Wall Follower
#python3 WallFollowerNode.py --ros-args -p use_sim_time:=true &

taskset -c 16-31 ros2 run rqt_robot_steering rqt_robot_steering &

echo "✅ Alle Systeme laufen."
wait