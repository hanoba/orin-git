#!/bin/bash

MY_PID=$$

cleanup() {
    echo -e "\n🛑 Beende NavigatorNode"
    pkill -P $MY_PID
    pkill -f "python3 NavigatorNode.py"
    exit 0
}

trap cleanup SIGINT SIGTERM

clear
echo "🚀 Starte System..."

cd /root/ros2
ros2 daemon start

# --- NAVIGATION ---
echo "🚀 Starte Navigator..."
python3 NavigatorNode.py

echo "✅ System läuft mit Ground Truth von der Bridge."
