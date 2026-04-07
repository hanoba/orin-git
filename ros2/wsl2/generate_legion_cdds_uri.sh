#!/bin/bash

# CYCLONEDDS_URI für LEGION Laptop generieren

# SSID-Abfrage
_SSID=$(netsh.exe wlan show interfaces | grep -i '^ *SSID' | cut -d: -f2 | tr -d ' \r')

if [ -n "$_SSID" ]; then
    export WLAN_NAME="$_SSID"
    
    # CYCLONEDDS_URI setzen
    if [ "$WLAN_NAME" == "AndroidHanoba" ]; then
        export CYCLONEDDS_URI=/home/harald/orin-git/ros2/wsl2/cdds_legion_garten.xml
    else
        export CYCLONEDDS_URI=/home/harald/orin-git/ros2/wsl2/cdds_legion_wohnung.xml
    fi
else
    echo "Fehler: SSID konnte nicht gefunden werden."
    return 1 2>/dev/null || exit 1
fi
