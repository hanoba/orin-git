import numpy as np
import matplotlib.pyplot as plt
import cmath
import math
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets
import time
from LidarLivePlot import Lidar


class Gate():
    def __init__(self, gateWidth, 
        startPointDist, 
        vonRechts, 
        maxWinkel=45, 
        freeRangeDist=6,
        segBasedDetection=False,
        segMinPoints=100,
        segThreshold=0.1
        ):
        # Torbreite 
        self.gateWidth = gateWidth              # Breite des Tores
        self.gateWidthMin = self.gateWidth - 0.15
        self.gateWidthMax = self.gateWidth + 0.40

        self.startPointDist = startPointDist    # Abstand des Startpunktes vom Tor
        self.vonRechts = vonRechts              # Torsuche von rechts beginnen
        self.maxWinkelRad = maxWinkel/180*np.pi # maximal Öffnungswinkel in Fahrtrichtung
        self.freeRangeDist = freeRangeDist
        self.minInsideGatePoints = 14           # Mindestanzahl der Messwerte im Tor
        self.segBasedDetection = segBasedDetection
        self.segMinPoints = segMinPoints        # minimum number of points per segments
        self.segThreshold = segThreshold        # maximaler Abstand zur Linie [m]

    def Detect(self, angle, radius, points):
        """Erkennt die breiteste „freie“ Winkelmenge im 360°‑LiDAR.

        Vorgehen
          1) Klassifiziere jeden Strahl als frei (1) oder belegt (0) via Schwellwert.
          2) Finde die längste zusammenhängende Sequenz von 1en. Zirkulär behandeln,
             daher wird die Liste zu sich selbst konkateniert.
          3) Liefere den Mittelindex und dessen Winkel (in Rad) zurück.

        Rückgabe
          (mid_angle, length) oder None, falls kein genügend breites Gate gefunden wird.
        """
        if self.segBasedDetection: 
            return self.SegBasedDetection(points)
            

        n = len(radius)

        # Suche ersten Punkt des Zaunes
        i0 = None
        for i in range(n):
            if radius[i] < self.freeRangeDist: 
                i0 = i
                break
        if i0 == None: 
            print ("Zaun nicht gefunden")       #HB
            return None
        
        # Suche Tor
        cur_len = 0
        i2 = 0
        margin = 8
        
        torBreite = 0
        found = False
        gateFreeRangeThresh = self.freeRangeDist
        for i in range(i0+1,n):
            d = radius[i]
            if d >= gateFreeRangeThresh:
                if cur_len == 0:
                    i1 = i-1-margin
                    gateFreeRangeThresh = radius[i1] + 0.5
                cur_len += 1
            else:
                if cur_len > 0: 
                    i2 = i+margin
                    gateFreeRangeThresh = self.freeRangeDist
            
                    if cur_len > self.minInsideGatePoints:
                        #print (f"{i1=}, {i2=}")
                        # Erster Pfosten als complexe Zahl
                        phi1 = angle[i1]
                        d1 = radius[i1]
                        pfosten1 = cmath.rect(d1, phi1)
    
                        # Zweiter Pfosten als complexe Zahl
                        phi2 = angle[i2]
                        d2 = radius[i2]
                        pfosten2 = cmath.rect(d2, phi2)
    
                        # die Breite des Tores muss zwischen gateWidthMin und gateWidthMax liegen
                        tor = pfosten2 - pfosten1
                        torBreite = abs(tor)
                        #print(f"{torBreite=}, {i1=}, {i2=}")
                        if torBreite <= self.gateWidthMax and torBreite >= self.gateWidthMin:
                            found = True
                            break
                    cur_len = 0
            
        if not found:
            print (f"Tor nicht gefunden {torBreite=}")       #HB
            return None  # , None
            
        # Winkel zum Mittelpunkt des Tores
        torMitte = (pfosten2 + pfosten1) / 2
        startPoint = torMitte + self.startPointDist * tor / torBreite * (1j if self.vonRechts else -1j)
        #print(f"{pfosten1=}, {pfosten2=}")
        return torMitte, startPoint, pfosten1, pfosten2

    def Preprocessing(self, angles, radius):
        # Winkel-Filter
        mask = (angles >= np.pi/2-self.maxWinkelRad) & (angles <= np.pi/2+self.maxWinkelRad)
        angles = angles[mask]
        radius = radius[mask]

        mask = (radius > 0.1) 
        angles = angles[mask]
        radius = radius[mask]

        radius = np.array(radius)

        if self.vonRechts:
            angles = np.flip(angles)
            radius = np.flip(radius)

        x = radius * np.cos(angles)
        y = radius * np.sin(angles)
        points = np.stack((x, y), axis=1)

        return angles, radius, points        

    # --- Abstand Punkt -> Linie ---
    def point_line_dist(self, p, a, b):
        # a,b: Endpunkte der Linie
        if np.allclose(a, b): return np.linalg.norm(p - a)
        return np.abs(np.cross(b - a, a - p)) / np.linalg.norm(b - a)

    # --- Rekursiver Split ---
    def split(self, points):
        if len(points) < self.segMinPoints:
            return []

        a, b = points[0], points[-1]
        dists = np.array([self.point_line_dist(p, a, b) for p in points])
        i_max = np.argmax(dists)
        d_max = dists[i_max]

        if d_max > self.segThreshold and i_max not in (0, len(points)-1):
            left = self.split(points[:i_max+1])
            right = self.split(points[i_max:])
            return left + right
        else:
            # nur akzeptieren, wenn genügend Punkte
            return [(a, b)] if len(points) >= self.segMinPoints else []

    def SegBasedDetection(self, points):
        segments = self.split(points)

        numSegs = len(segments)
        print(f"{numSegs} Liniensegmente erkannt.")
        for seg in segments:
            print(seg)

        if numSegs > 1:
            pfosten1 = segments[0][1][0] + 1j*segments[0][1][1]
            pfosten2 = segments[1][0][0] + 1j*segments[1][0][1]
            tor = pfosten2 - pfosten1
            torBreite = abs(tor)
            print(f"{torBreite=}")
            torMitte = (pfosten2 + pfosten1) / 2
            startPoint = torMitte + 1 * tor / torBreite * (-1j)
            if torBreite <= self.gateWidthMax and torBreite >= self.gateWidthMin:
                return torMitte, startPoint
            print(f"{torBreite=} passt nicht")       #HB
        else: print(f"{numSegs=} not enough segments found") 
        return None  # , None


# --- Plot-Setup --------------------------------------------------------------
app = QtWidgets.QApplication([])
pg.setConfigOptions(antialias=True)
win = pg.plot(title="YDLidar TG30 – Live-Scan")
win.setAspectLocked(True)
win.disableAutoRange()
win.setXRange(-2,2)  #HB(-10, 10)
win.setYRange(0, 2)   #10)
win.showGrid(x=True, y=True)
# --- Vollbildmodus aktivieren ---
win.showFullScreen()

curve = win.plot(pen=None, symbol='o', symbolSize=2, symbolBrush=(0, 255, 0))
curve2 = win.plot(pen=None, symbol='o', symbolSize=8, symbolBrush=(255, 0, 0))

# 
gate = Gate(
    gateWidth=0.275, 
    startPointDist=0.4, 
    maxWinkel = 80,
    vonRechts=True, 
    segBasedDetection=False,
    freeRangeDist=3.0)

#text = pg.TextItem(anchor=(0,1))
#win.getPlotItem().addItem(text)
#text.setPos(-5, 9)
#name = "Gate detection test"

lidar = Lidar()

try:
    while True:
        angles, radius = lidar.Scan()
        if len(angles) == 0:
            continue

        angles, radius, points = gate.Preprocessing(angles, radius)
        curve.setData(points)
        #print(points[:5])
        
        result = gate.Detect(angles, radius, points)
        if result != None: 
            torMitte, startPoint, pfosten1, pfosten2 = result
            curve2.setData(
                [torMitte.real, startPoint.real, pfosten1.real, pfosten2.real], 
                [torMitte.imag, startPoint.imag, pfosten1.imag, pfosten2.imag])

        #h=f"<div style=\"background-color:black; color:red; font-size:20pt;\">{name}</div>"
        #text.setHtml(h)
        QtWidgets.QApplication.processEvents()
except KeyboardInterrupt:
    pass
finally:
    print("\n⏹ Lidar gestoppt.")