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

# --- 1. NETZWERK-FIX (WICHTIG!) ---
# Damit CycloneDDS genug Ports in WSL2 findet und nicht abstürzt:
export ROS_DOMAIN_ID=15
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>500</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'

# Pfad zur Karte
MAP_YAML="/home/harald/orin-git/ros2/map/garten_map_10cm.yaml"
LASER_MAX_RANGE=20.0
AMCL_CONFIG_FILE=./config/amcl_config.yaml


clear
echo "🚀 Starte Nodes... Lidar Range = $LASER_MAX_RANGE m"

cd /home/harald/orin-git/ros2
ros2 daemon start

# Bridge (WICHTIG: keine Sim-Zeit hier)
python3 bridge.py --ros-args -p publish_odom_tf:=false   -p lidarRangeMax:=$LASER_MAX_RANGE &

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
  -p freq:=10.0 \
  -p odom_topic:=/rf2o_odom \
  -p init_pose_from_topic:=\"\"  &

#  -p tf_timeout:=0.2 \



# Compass Fusion Node (Kombiniert Laser-Speed + Isaac-Winkel -> Perfektes Odom & TF)
python3 CompassFusionNode.py --ros-args -p use_sim_time:=true &

echo "⏳ Warte kurz auf Odometrie-Stream..."
sleep 2


echo "🚀 Starte AMCL-Lokalisierung..."

# Map Server (lädt das Bild)
ros2 run nav2_map_server map_server \
    --ros-args \
    -p yaml_filename:=$MAP_YAML \
    -p use_sim_time:=true &

# AMCL (der Lokalisierer)
ros2 run nav2_amcl amcl \
    --ros-args          \
    --params-file $AMCL_CONFIG_FILE \
    -p laser_max_range:=$LASER_MAX_RANGE \
    -p use_sim_time:=true &

#    -r /particle_cloud:=/particle_cloud_pose_array 


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

echo "✅ System läuft! Fusion Odometrie & AMCL aktiv."

sleep 2

# WallFollower
python3 WallFollowerNode.py  --ros-args -p use_sim_time:=true 

wait
