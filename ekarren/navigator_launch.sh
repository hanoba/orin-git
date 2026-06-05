#!/bin/bash

# navigator_launch.sh

MY_PID=$$

cleanup() {
    echo -e "\n🛑 Beende alle Prozesse (Ground Truth Mode)..."
    pkill -P $MY_PID
    pkill -f "python3 SimMain.py"
    pkill -f "python3 viz.py"
    exit 0
}

trap cleanup SIGINT SIGTERM


# --- IP Adressen für viz.py  ---
export EKARREN_VIZ_IP1="192.168.178.104"    # IP Laptop Wohnung
export EKARREN_VIZ_IP2="127.0.0.1"          # IP Desktop Wohnung

# Konstanten
LIDAR_X=0.8
SIM_TIME=false

clear
echo "🚀 Starte Host-Simulation des E-Karren..."

cd /home/harald/orin-git/ekarren
pkill -9 -f _ros2_daemon
ros2 daemon start

# --- 2. CORE KOMPONENTEN ---

# PyGame-Simulator
python3 SimMain.py

# Pygame-Visualizer (ersetzt rviz2)
###python3 viz.py &

echo "✅ Host-Simulation läuft."
wait
