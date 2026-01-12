#!/bin/bash

# Wir speichern die aktuelle Prozess-ID des Skripts
MY_PID=$$

cleanup() {
    echo -e "\n🛑 Beende alle ROS-Prozesse..."
    # Wir killen alle Prozesse, die MY_PID als Vater (PPID) haben
    pkill -P $MY_PID
    
    # Sicherheitshalber alle Python-Nodes dieses Projekts
    pkill -f "python3 bridge.py"
    pkill -f "python3 WallFollowerNode.py"
    pkill -f "python3 yaw_to_imu.py"     # NEU: Der Yaw-Converter
    pkill -f "ekf_node"                   # NEU: Der Kalman Filter
    
    pkill -f "static_transform_publisher"
    pkill -f "rviz2"
    exit 0
}

# Trap auf SIGINT (Strg+C)
trap cleanup SIGINT SIGTERM

# --- 1. NETZWERK-FIX ---
export ROS_DOMAIN_ID=15
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>500</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'

# Pfade & Configs
MAP_YAML="/home/harald/orin-git/ros2/map/garten_map_10cm.yaml"
LASER_MAX_RANGE=20.0
AMCL_CONFIG_FILE=./config/amcl_config_ok.yaml
EKF_CONFIG_FILE=./config/ekf_config.yaml

clear
echo "🚀 Starte Nodes... Lidar Range = $LASER_MAX_RANGE m"

cd /home/harald/orin-git/ros2
ros2 daemon start

# Bridge (WICHTIG: keine Sim-Zeit hier, falls sie extern läuft, sonst anpassen)
# publish_odom_tf:=false ist hier wichtig, damit die Bridge nicht stört
python3 bridge.py --ros-args -p publish_odom_tf:=false -p lidarRangeMax:=$LASER_MAX_RANGE &

# 1. Statischer Transform (Lidar sitzt 40cm vor base_link)
ros2 run tf2_ros static_transform_publisher 0.4 0 0 0 0 0 base_link lidar --ros-args -p use_sim_time:=true &

# --- 2. ODOMETRIE & FUSION ---

# A) RF2O Laser Odometrie
# WICHTIG: publish_tf:=false, da der EKF das TF übernimmt!
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node \
  --ros-args \
  --log-level error \
  -p laser_scan_topic:=/scan \
  -p base_frame_id:=base_link \
  -p odom_frame_id:=odom \
  -p use_sim_time:=true \
  -p freq:=10.0 \
  -p odom_topic:=/rf2o_odom \
  -p publish_tf:=false \
  -p init_pose_from_topic:=\"\" &

# B) Yaw-Converter (Der Adapter Node aus der vorherigen Antwort)
# Er wandelt das Isaac Float Topic in /imu_data um
# Passen Sie 'input_topic' an den Namen in Ihrer bridge.py an!
python3 YawToImuNode.py --ros-args -p use_sim_time:=true -p input_topic:=/compass_heading &

# C) Extended Kalman Filter (EKF)
ros2 run robot_localization ekf_node --ros-args --params-file $EKF_CONFIG_FILE &

echo "⏳ Warte kurz auf Fusion..."
sleep 2

echo "🚀 Starte AMCL-Lokalisierung..."

# Map Server
ros2 run nav2_map_server map_server \
    --ros-args \
    -p yaml_filename:=$MAP_YAML \
    -p use_sim_time:=true &

# AMCL
ros2 run nav2_amcl amcl \
    --ros-args          \
    --params-file $AMCL_CONFIG_FILE \
    -p laser_max_range:=$LASER_MAX_RANGE \
    -p transform_tolerance:=1.0 \
    -p use_sim_time:=true &
# Hinweis: transform_tolerance erhöht, wie besprochen!

# Lifecycle Manager
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p node_names:="['map_server', 'amcl']" \
  -p autostart:=true \
  -p use_sim_time:=true &
  
# RViz
rviz2 -d config/amcl.rviz --ros-args -p use_sim_time:=true &

echo "⏳ Warte 3s auf Initialisierung..."
sleep 3

# Initial Pose setzen
ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}
}" 

echo "✅ System läuft! EKF Fusion aktiv."

sleep 2

# WallFollower
python3 WallFollowerNode.py  --ros-args -p use_sim_time:=true 

wait