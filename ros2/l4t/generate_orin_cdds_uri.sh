#!/bin/bash

# CYCLONEDDS_URI für Orin-NX generieren

# SSID-Abfrage
_SSID=$(nmcli -t -f active,ssid dev wifi | grep -E '^ja:|^yes:' | cut -d: -f2 | head -n 1)

if [ -n "$_SSID" ]; then
    export WLAN_NAME="$_SSID"
    
    # CYCLONEDDS_URI setzen
    if [ "$WLAN_NAME" == "AndroidHanoba" ]; then
        export CYCLONEDDS_URI=/root/ros2/l4t/cdds_orin_garten.xml
    else
        export CYCLONEDDS_URI=/root/ros2/l4t/cdds_orin_wohnung.xml
    fi
else
    echo "Fehler: SSID konnte nicht gefunden werden."
    return 1 2>/dev/null || exit 1
fi
