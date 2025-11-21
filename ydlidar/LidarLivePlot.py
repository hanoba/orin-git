#!/usr/bin/env python3
import ydlidar
import numpy as np
#import matplotlib.pyplot as plt
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets
import time

class Lidar():
    def __init__(self):
        # === Parameter anpassen ===
        PORT = "/dev/ttyUSB0"
        BAUD = 512000           # TG30 nutzt 512000 Baud

        # === Lidar initialisieren ===
        self.laser = ydlidar.CYdLidar()
        self.laser.setlidaropt(ydlidar.LidarPropSerialPort, PORT)
        self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, BAUD)
        self.laser.setlidaropt(ydlidar.LidarPropFixedResolution, False)
        self.laser.setlidaropt(ydlidar.LidarPropReversion, False)
        self.laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
        self.laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TOF)
        self.laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)

        ret = self.laser.initialize()
        if not ret:
            print("Fehler: Initialisierung fehlgeschlagen")
            exit(1)

        if not self.laser.turnOn():
            print("Fehler: Laser konnte nicht gestartet werden")
            exit(1)

        print("LIDAR läuft – Strg+C zum Beenden")
        self.scan = ydlidar.LaserScan()


    def Scan(self):
        if self.laser.doProcessSimple(self.scan):
            # Rohdaten in Winkel/Entfernung umrechnen
            # Umrechnung in Roboterkoordinationsystem (x zeigt in Fahrtrichtung)
            angles =  np.array([(np.pi/2 - p.angle) % (2*np.pi) for p in self.scan.points])
            radius  = np.array([p.range for p in self.scan.points])
            return angles, radius
        else:
            time.sleep(0.05)
            return [], []
            
    def Close(self):
        # === Aufräumen ===
        self.laser.turnOff()
        self.laser.disconnecting()


class LidarPlot():
    def __init__(self):
        app = QtWidgets.QApplication([])
        pg.setConfigOptions(antialias=True)
        self.win = pg.plot(title="YDLidar TG30 – Live-Scan")
        self.win.setAspectLocked(True)
        self.win.setXRange(-10, 10)
        self.win.setYRange(-10, 10)
        self.win.showGrid(x=True, y=True)
        # --- Vollbildmodus aktivieren ---
        self.win.showFullScreen()
        self.curve = self.win.plot(pen=None, symbol='o', symbolSize=2, symbolBrush=(0, 255, 0))

    def Points(self, points):
        self.curve.setData(points)
        QtWidgets.QApplication.processEvents()

    def Polar(self, angles, radius):
        x = radius * np.cos(angles)
        y = radius * np.sin(angles)
        points = np.stack((x, y), axis=1)
        self.Points(points)

# --- Hauptprogramm ---
if __name__ == "__main__":
    plot = LidarPlot()
    try:
        lidar = Lidar()
        while True:
            angles, radius = lidar.Scan()
            plot.Polar(angles, radius)
    except KeyboardInterrupt:
        lidar.Close()
        print("\nBeendet durch Benutzer.")
