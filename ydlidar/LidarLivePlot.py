#!/usr/bin/env python3
import ydlidar
import numpy as np
import matplotlib.pyplot as plt
import time

# === Parameter anpassen ===
PORT = "/dev/ttyUSB0"
BAUD = 512000           # TG30 nutzt 512000 Baud
SAMPLE_COUNT = 360      # 1°-Raster

# === Lidar initialisieren ===
laser = ydlidar.CYdLidar()
laser.setlidaropt(ydlidar.LidarPropSerialPort, PORT)
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, BAUD)
laser.setlidaropt(ydlidar.LidarPropFixedResolution, False)
laser.setlidaropt(ydlidar.LidarPropReversion, False)
laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TOF)
laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)

ret = laser.initialize()
if not ret:
    print("Fehler: Initialisierung fehlgeschlagen")
    exit(1)

if not laser.turnOn():
    print("Fehler: Laser konnte nicht gestartet werden")
    exit(1)

print("LIDAR läuft – Strg+C zum Beenden")

# === Matplotlib-Setup ===
plt.ion()
fig = plt.figure(figsize=(6,6))
ax = fig.add_subplot(111, polar=True)
points, = ax.plot([], [], '.', markersize=2)
ax.set_ylim(0, 12)   # max. 12 m Reichweite

try:
    scan = ydlidar.LaserScan()
    while True:
        if laser.doProcessSimple(scan):
            # Rohdaten in Winkel/Entfernung umrechnen
            angles = np.array([p.angle for p in scan.points])
            dists  = np.array([p.range for p in scan.points])
            points.set_data(angles, dists)
            ax.set_title("YDLidar TG30 Live-Scan")
            plt.pause(0.001)
        else:
            time.sleep(0.05)
except KeyboardInterrupt:
    print("\nBeendet durch Benutzer.")

# === Aufräumen ===
laser.turnOff()
laser.disconnecting()
plt.close()
