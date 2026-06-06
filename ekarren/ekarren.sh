#!/bin/bash
# File: ekarren.sh
# Startet E-Karren auf Orin-NX  (echte Hardware)

cd /home/harald/orin-git/ekarren/
clear
echo "🚀 Starte E-Karren..."

# IP-Adressen setzen
source GenerateVizIpAddr.sh

# E-Karren Hauptprogramm starten
python3 eKarrenMain.py
