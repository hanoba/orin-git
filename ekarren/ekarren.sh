#!/bin/bash
# File: ekarren.sh
# Startet E-Karren-Software auf Orin-NX  (echte Hardware)

cd /home/harald/orin-git/ekarren/
clear
echo "🚀 Starte E-Karren-Software..."

# IP-Adressen setzen
source GenerateVizIpAddr.sh

# E-Karren Hauptprogramm starten
python3 eKarrenNew.py $1 $2
