#!/bin/bash

# eksim.sh

# MY_PID=$$
# 
# cleanup() {
#     echo -e "\n🛑 Beende alle Prozesse..."
#     pkill -P $MY_PID
#     pkill -f "python3 SimMain.py"
#     exit 0
# }
# 
# trap cleanup SIGINT SIGTERM


# --- IP Adressen für viz.py  ---
export EKARREN_VIZ_IP1="192.168.178.104"    # IP Laptop Wohnung
export EKARREN_VIZ_IP2="127.0.0.1"          # IP Desktop Wohnung

clear
echo "🚀 Starte Host-Simulation des E-Karrens..."

cd /home/harald/orin-git/ekarren

# PyGame-Simulator
python3 SimMain.py $1 $2
