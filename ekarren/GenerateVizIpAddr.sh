#!/bin/bash

# CYCLONEDDS_URI und EKARREN_VIZ_IP1/2 für Orin-NX generieren

# SSID-Abfrage
_SSID=$(nmcli -t -f active,ssid dev wifi | grep -E '^ja:|^yes:' | cut -d: -f2 | head -n 1)

if [ -n "$_SSID" ]; then
    export WLAN_NAME="$_SSID"
    
    # CYCLONEDDS_URI setzen
    if [ "$WLAN_NAME" == "AndroidHanoba" ]; then
        export EKARREN_VIZ_IP1="192.168.20.41"      # IP LEGION Garten
        export EKARREN_VIZ_IP2="192.168.20.101"     # IP HBA004 Garten
    else
        export EKARREN_VIZ_IP1="192.168.178.104"    # IP LEGION Wohnung
        export EKARREN_VIZ_IP2="192.168.178.42"     # IP AZ-KENKO Wohnung
    fi
else
    echo "Fehler: SSID konnte nicht gefunden werden."
    return 1 2>/dev/null || exit 1
fi
