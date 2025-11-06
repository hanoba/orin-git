#!/usr/bin/env python3
# coding: utf-8

import numpy as np
import cmath
import math
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets
import time
from RobotLib import Lidar, Gate
from Rosmaster_Lib import Rosmaster


# --- Plot-Setup --------------------------------------------------------------
app = QtWidgets.QApplication([])
pg.setConfigOptions(antialias=True)
win = pg.plot(title="YDLidar TG30 – Live-Scan")
win.setXRange(0,2) 
win.setYRange(-2, 2)  
win.setAspectLocked(True)
win.disableAutoRange()
win.showGrid(x=True, y=True)
# --- Vollbildmodus aktivieren ---
win.showFullScreen()

curve = win.plot(pen=None, symbol='o', symbolSize=2, symbolBrush=(0, 255, 0))
curve2 = win.plot(pen=None, symbol='o', symbolSize=8, symbolBrush=(255, 0, 0))

# 
gate = Gate(
    gateWidthMin=0.275-0.1, 
    gateWidthMax=0.4, 
    startPointDist=0.4, 
    maxWinkel = 80,
    gateIndexMargin=8,
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

        QtWidgets.QApplication.processEvents()
except KeyboardInterrupt:
    pass
finally:
    print("\n⏹ Lidar gestoppt.")