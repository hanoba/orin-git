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

# Macht Kompass-Kalibrierung (Service: Kompass_Kalibrierung)
python3 CompassCalibrationNode.py &

echo "✅ System läuft mit Ground Truth von der Bridge."
