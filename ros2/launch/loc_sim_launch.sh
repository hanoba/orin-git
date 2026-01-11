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
ros2 daemon start

# Bridge (WICHTIG: keine Sim-Zeit hier)
python3 bridge.py --ros-args -p publish_odom_tf:=false &

# WallFollower
python3 WallFollowerNode.py  --ros-args -p use_sim_time:=true &

# 1. Statischer Transform (Lidar sitzt 40cm vor base_link)
#ros2 run tf2_ros static_transform_publisher 0.4 0 0 0 0 0 base_link lidar --ros-args -p use_sim_time:=true &
ros2 run tf2_ros static_transform_publisher --x 0.4 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
     --frame-id base_link --child-frame-id lidar --ros-args -p use_sim_time:=true &
#ros2 run tf2_ros static_transform_publisher 0.4 0 0 0 0 0 base_link lidar --ros-args -p use_sim_time:=true --publish-on-static false &

# 2. DER FEHLENDE TEIL: Verbindung von odom zu base_link
# ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link --ros-args -p use_sim_time:=true &

ros2 run rf2o_laser_odometry rf2o_laser_odometry_node \
  --ros-args \
  -p laser_scan_topic:=/scan \
  -p base_frame_id:=base_link \
  -p odom_frame_id:=odom \
  -p use_sim_time:=true \
  -p tf_timeout:=0.2 \
  -p freq:=10.0 \
  -p init_pose_from_topic:=\"\"  &

#  --log-level debug

  
# 2. RViz mit Sim-Zeit
rviz2 -d config/Localization.rviz --ros-args -p use_sim_time:=true &

# 4. Slam Toolbox (WICHTIG: Sim-Zeit hier ergänzt)
ros2 run slam_toolbox localization_slam_toolbox_node --ros-args --params-file ./config/localization.yaml  -p use_sim_time:=true 

