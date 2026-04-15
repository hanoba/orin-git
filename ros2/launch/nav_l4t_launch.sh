#!/bin/bash

# File: nav_l4t_launch.sh

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
echo "🚀 Starte verteilte Simulation auf Orin-NX..."

cd /root/ros2
ros2 daemon start

# --- NAVIGATION ---
echo "🚀 Starte Navigator..."
python3 NavigatorNode.py &

# Macht Kompass-Kalibrierung (Service: Kompass_Kalibrierung)
python3 CompassCalibrationNode.py &

echo "✅ Verteilte Simulation auf Orin-NX gestartet."

wait
