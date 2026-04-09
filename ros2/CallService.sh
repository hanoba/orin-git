#!/bin/bash

if [ -z "$1" ]; then
    echo "Bitte gib einen der folgenden Service-Namen an:"
    echo "    Kompass_Kalibrierung"
    echo "    Fahre_in_den_Garten"
    echo "    Fahre_hinters_Haus"
    echo "    Fahre_in_den_Wald"
    echo "    Fahre_zum_Schuppen"
    echo "    Localization"
    echo "    Bestimme_YawOffset"
    echo "    Mowing"
    echo "    Stop"
    exit 1
fi

ros2 service call /$1 std_srvs/srv/Trigger {}
