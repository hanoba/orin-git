#!/bin/bash

# ekarren_l4t_launch.sh

LIDAR_X=0.8

if [ -z "$1" ]; then
    DEVICE="eKarren"              # E-Karren (echte Hardware)
elif [ "$1" == "eKarrenEmulator" ]; then
    DEVICE="eKarrenEmulator"      # Rosmaster-Roboter emuliert E-Karren. Benötigt: sudo service start eKarrenEmulator auf Orin-NX
elif [ "$1" == "eKarrenPC" ]; then
    DEVICE="eKarrenPC"            # Fahrbefehle werden via UDP an PC gesendet. Benötigt: orin-git/eKarrenCtrl/UdpHostTest.py auf PC
else
    echo "Unbekannter DEVICE: $DEVICE"
    echo "Usage: bash ekarren_l4t_launch.sh [eKarrenEmulator|eKarrenPC]"
    exit 1    
fi
#

MY_PID=$$

cleanup() {
    echo "Beende Prozesse im Container..."
    # Alle Kindprozesse beenden
    kill -SIGINT $(jobs -p) 2>/dev/null
    sleep 1
    # Sicherstellen, dass alles weg ist
    exit 0
}

trap cleanup SIGINT SIGTERM

clear
echo "Starte E-Karren..."

cd /home/harald/orin-git/ros2
ros2 daemon start

# Statischer Transform: Lidar zu base_link
ros2 run tf2_ros static_transform_publisher $LIDAR_X 0 0 0 0 0 base_link lidar &

# Odom ist jetzt fest mit dem Roboter verbunden (da keine Encoder vorhanden)
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link &

# E-Karren Node (publiziert /scan und /compass_heading. Steuert E-Karren based on subscribed /cmd_vel)
python3 eKarrenNode.py  --ros-args -p device:=$DEVICE &

# Main Node zur Steuerung des E-Karrens
python3 NavigatorNode.py --ros-args -p publish_odom_tf:=True &

# Macht Kompass-Kalibrierung (Service: Kompass_Kalibrierung)
python3 CompassCalibrationNode.py &

#ros2 topic echo /cmd_vel

wait
