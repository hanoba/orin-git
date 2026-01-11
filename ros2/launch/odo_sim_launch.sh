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
python3 bridge.py &
ros2 run tf2_ros static_transform_publisher 0.4 0 0 0 0 0 base_link lidar --ros-args -p use_sim_time:=true &

#ros2 run topic_tools relay /scan /scan_clean --ros-args -p reliability:=best_effort &
###ros2 run topic_tools relay /scan /scan_clean --ros-args -p qos_overrides./scan.reliability:=best_effort &

#python3 WallFollowerNode.py  --ros-args -p use_sim_time:=true &

#ros2 run rf2o_laser_odometry rf2o_laser_odometry_node --ros-args \
#    -p laser_scan_topic:=/scan \
#    -p odom_topic:=/odom \
#    -p publish_tf:=True \
#    -p base_frame_id:=base_link \
#    -p odom_frame_id:=odom \
#    -p freq:=10.0 \
#    -p use_sim_time:=true \
#    --log-level debug &

#rviz2 -d config/Lidar.rviz  --ros-args -p use_sim_time:=true &

wait
