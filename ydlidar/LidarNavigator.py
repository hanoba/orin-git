#!/usr/bin/env python3
# coding: utf-8

import numpy as np
import cmath
import math
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets
import time
from RobotLib import Lidar, Gate, RobotController
from Rosmaster_Lib import Rosmaster


class Robot():
    def __init__(self):
        # Roboter über USB-Port initialisieren
        self.bot = Rosmaster(com="/dev/ttyCH341USB0", debug=False)
        self.bot.create_receive_threading()  # Empfangsthread für Statuswerte starten
        self.Stop()

        # Kurze Pause für Initialisierung
        time.sleep(.1)
        # Startsignal: drei kurze Pieptöne
        for i in range(3):
            self.bot.set_beep(60)
            time.sleep(.2)

        # Systeminformationen anzeigen
        # Abfrage der Firmware-Version und Batteriespannung des Roboters zur Statusanzeige
        print("Version:", self.bot.get_version())
        print("Vbat:", self.bot.get_battery_voltage())

        # Optionale Lichtsteuerung (auskommentiert)
        # Effect =[0, 6], 0: stop light effect, 1: running light, 2: running horse light,
        #                 3: breathing light, 4: gradient light, 5: starlight, 6: power display 
        #                 Speed =[1, 10], the smaller the value, the faster the speed changes
        # bot.set_colorful_effect(effect=2, speed=5)
    def SetSpeed(self, vLinear, vAngular):
        # Steuerbefehl an Roboter senden (Vorwärts-/Rückwärts- und Drehbewegung)
        # v_x=[-0.7, 0.7] m/s, v_y=[-0.7, 0.7] m/s, v_z=[-3.2, 3.2] rad/sec
        vAngularMax = 2*3.14/8  # eine Umdrehung in 4 Sekunden
        if vAngular > vAngularMax: vAngular = vAngularMax
        elif vAngular < -vAngularMax: vAngular = -vAngularMax
        
        vLinearMax = 0.1  # m/s
        if vLinear > vLinearMax: vLinear = vLinearMax
        elif vLinear < -vLinearMax: vLinear = -vLinearMax
        
        self.bot.set_car_motion(v_x=vLinear, v_y=0, v_z=vAngular)
        # Umrechnung in Radspeed‑Kommandos (Differentialfahrwerk)
        #vl = v_cmd - omega_cmd * (self.wheelBase / 2.0)
        #vr = v_cmd + omega_cmd * (self.wheelBase / 2.0)

    def SetColor(self, red, green, blue):
        # red,green,blue=[0, 255]，表示颜色RGB值。
        # RGB programmable light belt control, can be controlled individually or collectively, before control need to stop THE RGB light effect.
        # Led_id =[0, 13], control the CORRESPONDING numbered RGB lights;  Led_id =0xFF, controls all lights.
        # Red,green,blue=[0, 255], indicating the RGB value of the color.
        led_id = 0xFF
        self.bot.set_colorful_lamps(led_id, red, green, blue)

    def Stop(self):
        self.bot.set_car_motion(v_x=0, v_y=0, v_z=0)
        self.bot.set_colorful_lamps(255, 0, 0, 0)
      

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
    gateWidthMin=0.42-0.2, 
    gateWidthMax=0.42+0.2, 
    startPointDist=0.6, 
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
robot = Robot()
robotController = RobotController(
            robot, 
            gate,
            gateReachedThreshold=0.3,           # Schwellwert für Erreichen des Tores [m]
            startPointThreshold=0.1,            # Schwellwert für Erreichen Startpunktes [m]
            baseSpeed = 0.2,                    # Basisfahrgeschwindigkeit [m/s]#
            kHeading = 2.2                      # Proportionalgain auf den Richtungsfehler
)

try:
    while True:
        angles, radius = lidar.Scan()
        if len(angles) == 0:
            continue

        angles, radius, points = gate.Preprocessing(angles, radius)

        # Autopilot
        dt = 0.1
        result = robotController.Run(angles, radius, dt)

        curve.setData(points)
        
        #result = gate.Detect(angles, radius, points)
        if result != None: 
            torMitte, startPoint, pfosten1, pfosten2 = result
            curve2.setData(
                [torMitte.real, startPoint.real, pfosten1.real, pfosten2.real], 
                [torMitte.imag, startPoint.imag, pfosten1.imag, pfosten2.imag])

        if robotController.Ready(): 
            break
        QtWidgets.QApplication.processEvents()
except KeyboardInterrupt:
    pass
finally:
    robot.Stop()
    print("\nAutopilot gestoppt.")