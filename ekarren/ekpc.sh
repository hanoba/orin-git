#!/bin/bash
# File: ekpc.sh
# Startet E-Karren-Software auf Orin-NX  (echte Hardware)

cd /home/harald/orin-git/ekarren/
clear
echo "🚀 Starte E-Karren-Software..."

# IP-Adressen setzen
source GenerateVizIpAddr.sh

# E-Karren-Software starten für Tests mit PC
# Usage: ekpc [<taskName>]
# <taskName>:
#     None (default)
#     Localization
#     FastLocalization
#     Mowing
#     Fahre_zum_Schuppen
#     Fahre_in_den_Wald
#     Fahre_in_den_Garten
#     Fahre_hinters_Haus
#     Bestimme_YawOffset
#     Test
python3 eKarrenMain.py eKarrenPC $1
