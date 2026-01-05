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

clear
echo "🚀 Starte Nodes..."

cd /home/harald/orin-git/ros2
ros2 daemon start

# Bridge (WICHTIG: keine Sim-Zeit hier)
python3 bridge.py --ros-args -p publish_odom_tf:=false   -p lidarRangeMax:=20.0 &

# 1. Statischer Transform (Lidar sitzt 40cm vor base_link)
ros2 run tf2_ros static_transform_publisher 0.4 0 0 0 0 0 base_link lidar --ros-args -p use_sim_time:=true &

# RF2O Laser Odometrie (Erzeugt odom -> base_link)
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node \
  --ros-args \
  --log-level error  \
  -p laser_scan_topic:=/scan \
  -p base_frame_id:=base_link \
  -p odom_frame_id:=odom \
  -p use_sim_time:=true \
  -p tf_timeout:=0.2 \
  -p freq:=10.0 \
  -p init_pose_from_topic:=\"\"  &

MAP_YAML="/home/harald/orin-git/ros2/map/garten_map_10cm.yaml"

echo "🚀 Starte AMCL-Lokalisierung..."

# Map Server (lädt das Bild)
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$MAP_YAML -p use_sim_time:=true &

# AMCL (der Lokalisierer)
ros2 run nav2_amcl amcl \
    --ros-args          \
    --params-file ./config/amcl_config_ok.yaml \
    -r /particle_cloud:=/particle_cloud_pose_array \
    -p use_sim_time:=true &

# Lifecycle Manager (WICHTIG: Er schaltet Map Server und AMCL erst scharf)
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p node_names:="['map_server', 'amcl']" \
  -p autostart:=true \
  -p use_sim_time:=true &
  
# RViz mit Sim-Zeit
rviz2 -d config/amcl.rviz --ros-args -p use_sim_time:=true &

echo "⏳ Warte 3s auf Initialisierung..."
sleep 3

# Initial Pose setzen (damit er weiß, wo er auf deiner perfekten Karte steht)
# Hier 0,0,0 - passe das an, falls dein Startpunkt woanders liegt
# Orientation "w: 1.0" bedeutet Ausrichtung in x-Richtung - Theta=0°
ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}
}" 

echo "✅ AMCL ist aktiv!"

sleep 3

# WallFollower
python3 WallFollowerNode.py  --ros-args -p use_sim_time:=true 

wait
