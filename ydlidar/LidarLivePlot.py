#!/usr/bin/env python3
import ydlidar
import numpy as np
#import matplotlib.pyplot as plt
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets
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


# --- Plot-Setup --------------------------------------------------------------
app = QtWidgets.QApplication([])
pg.setConfigOptions(antialias=True)
win = pg.plot(title="YDLidar TG30 – Live-Scan")
win.setAspectLocked(True)
win.setXRange(-10, 10)
win.setYRange(-10, 10)
win.showGrid(x=True, y=True)
# --- Vollbildmodus aktivieren ---
win.showFullScreen()

curve = win.plot(pen=None, symbol='o', symbolSize=2, symbolBrush=(0, 255, 0))

try:
    scan = ydlidar.LaserScan()
    while True:
        if laser.doProcessSimple(scan):
            # Rohdaten in Winkel/Entfernung umrechnen
            ang270 = 270/180*np.pi
            angles =  np.array([ang270 - p.angle for p in scan.points])
            radius  = np.array([p.range for p in scan.points])

            #angles = (ang270-angles)

            x = radius * np.cos(angles)
            y = radius * np.sin(angles)
            points = np.stack((x, y), axis=1)

            curve.setData(points)
            QtWidgets.QApplication.processEvents()

        else:
            time.sleep(0.05)
except KeyboardInterrupt:
    print("\nBeendet durch Benutzer.")

# === Aufräumen ===
laser.turnOff()
laser.disconnecting()
plt.close()
