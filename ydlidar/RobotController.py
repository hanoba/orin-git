#!/usr/bin/env python3

import math
import cmath


# LiDAR‑Parameter
LIDAR_COUNT = 180      #HB 360       # Anzahl der Strahlen (1° Raster)
LIDAR_MAX_RANGE = 320.0*2            # maximale Messdistanz in Pixeln
LIDAR_NOISE_STD = 0.5                # Gauß‑Rauschen (σ) auf Distanzmessung

# Roboterkinematik
ROBOT_RADIUS = 16                    # nur für Zeichnung/Kollision (Kreis)
WHEEL_BASE = 2 * ROBOT_RADIUS        # Radabstand (vereinfacht)
MAX_WHEEL_SPEED = 120.0              # Sättigung der Radspeed‑Kommandos [px/s]
BASE_SPEED = 70.0                    # Basisfahrgeschwindigkeit [px/s]

# Regler‑Gains
K_HEADING = 2.2                      # Proportionalgain auf den Richtungsfehler
K_DISTANCE = 0.8                     # aktuell nicht genutzt, belassen für Experimente

# Gate‑Detektion
GATE_MIN_ANGULAR_WIDTH_DEG = 14      # minimale zusammenhängende freie Winkelbreite
GATE_FREE_RANGE_THRESH = LIDAR_MAX_RANGE * 0.92  # Schwelle ab der ein Strahl „frei“ gilt
GATE_REACHED_THRE = 30               # Schwellwert für Erreichen des Tores
START_POINT_REACHED_THRE = 10        # Schwellwert für Erreichen Startpunktes
TARGET_ANGLE_REACHED_THRE = 16       # Schwellwert für Ausrichtung zum Ziel

# Zustände der einfachen Zustandsmaschine
STATE_SEARCH = 0                     # Suchen / „patrouillieren“
STATE_GOTO_START = 1                 # Ausrichten auf Startpunkt und zufahren
STATE_ALIGN_AND_GO = 2               # Ausrichten auf Gate und zufahren
STATE_GATE_REACHED = 3               # Distance to gate less than GATE_REACHED_THRE
STATE_DONE = 4                       # hinter dem Tor angehalten



class RobotController()
    def __init__(self):
        self.robotState = STATE_SEARCH
        self.timeOut = 10
        
    def GetState(self):
        return f"{['SEARCH','GOTO_START','ALIGN&GO','GATE_REACHED','DONE'][self.robotState]}  "

    def Run(lidar_hits, dt):
        """Einfache Zustandssteuerung.

        SEARCH:         Dreht sich langsam, bis das Tor erkannt wurde.
        GOTO_START:     Fährt zum Startpunkt
        ALIGN&GO:       Regelt die Ausrichtung auf den Mittelpunkt des Tores und fährt vorwärts.
        GATE_REACHED:   Der Abstand zum Tor ist kleiner als 30cm
        DONE:           Stoppt, kurze Zeit nachdem das Tor erreicht wurde.
        """
        global timeOut, robotState
        if self.robotState == STATE_DONE:
            robot.set_wheels(0, 0)
            return None

        
        if self.robotState == STATE_SEARCH:
            # roboter drehen um das Panorama zu „scannen“
            robot.set_wheels(-20, 20)       # Nur Drehung
            result = detect_gate(lidar_hits)
            # Beim ersten validen Gate wechseln wir in den Ausrichtungs/Fahrt‑Modus
            if result is not None:
                torMitte, startPoint, pfosten1, pfosten2 = result
                robot.target_angle = cmath.phase(torMitte)  #HB[0]  # Winkel IN ROBOTERKOORDINATEN
                self.robotState = STATE_GOTO_START
                return torMitte, startPoint, pfosten1, pfosten2

        elif self.robotState == STATE_GOTO_START:
            # Aktualisiere Zielwinkel laufend, falls Gate sich geometrisch verschiebt
            result = detect_gate(lidar_hits)
            if result is not None:
                torMitte, startPoint, pfosten1, pfosten2 = result
                if abs(startPoint) > START_POINT_REACHED_THRE:
                    robot.target_angle = cmath.phase(startPoint)  #HB [0]

                    # Fehler zwischen Blickrichtung und Gate‑Mittelwinkel
                    # Hinweis: Da die LiDAR‑Winkel relativ zu theta erzeugt wurden, ist robot.target_angle
                    # bereits im Roboter‑Frame. 
                    #HB err = wrap_angle(gate[0] - robot.theta) 
                    err = wrap_angle(robot.target_angle - robot.theta*0) 
                    #print(f"err={G(err)}°")  #HB
                    omega_cmd = K_HEADING * err * 2 #HB
                    HB=0.0
                    if err<0: omega_cmd -= HB
                    else: omega_cmd += HB
                    if numSteps < 20: print(f"{robot.target_angle=}")   #HB
                    v_cmd = BASE_SPEED*1 #HB  # konstante Vorwärtsfahrt, Stabilität via Heading‑Regelung
                    if abs(G(robot.target_angle)) > TARGET_ANGLE_REACHED_THRE: v_cmd = 0
                    # Umrechnung in Radspeed‑Kommandos (Differentialfahrwerk)
                    vl = v_cmd - omega_cmd * (WHEEL_BASE / 2.0)
                    vr = v_cmd + omega_cmd * (WHEEL_BASE / 2.0)
                    robot.set_wheels(vl, vr)
                    #HB robot.set_wheels(0, 0)
                else: 
                    # start point reached
                    self.robotState = STATE_ALIGN_AND_GO
                    robot.set_wheels(0, 0)
                return torMitte, startPoint, pfosten1, pfosten2
        
        elif self.robotState == STATE_ALIGN_AND_GO:
            # Aktualisiere Zielwinkel laufend, falls Gate sich geometrisch verschiebt
            result = detect_gate(lidar_hits)
            if result is not None:
                torMitte, startPoint, pfosten1, pfosten2 = result
                robot.target_angle = cmath.phase(torMitte)  #HB [0]

                # Fehler zwischen Blickrichtung und Gate‑Mittelwinkel
                # Hinweis: Da die LiDAR‑Winkel relativ zu theta erzeugt wurden, ist robot.target_angle
                # bereits im Roboter‑Frame. 
                err = robot.target_angle
                #print(f"err={G(err)}°")  #HB
                omega_cmd = K_HEADING * err
                HB=0.0
                if err<0: omega_cmd -= HB
                else: omega_cmd += HB
                if numSteps < 20: print(f"{robot.target_angle=}")   #HB
                v_cmd = BASE_SPEED*1 #HB  # konstante Vorwärtsfahrt, Stabilität via Heading‑Regelung
                if abs(G(robot.target_angle)) > TARGET_ANGLE_REACHED_THRE: v_cmd = 0
                # Umrechnung in Radspeed‑Kommandos (Differentialfahrwerk)
                vl = v_cmd - omega_cmd * (WHEEL_BASE / 2.0)
                vr = v_cmd + omega_cmd * (WHEEL_BASE / 2.0)
                robot.set_wheels(vl, vr)
                #HB robot.set_wheels(0, 0)
                timeOut = 10
                # Stop‑Kriterium: geringer Abstand zum Tor
                if abs(torMitte) <= GATE_REACHED_THRE:
                    self.robotState = STATE_GATE_REACHED
                return torMitte, startPoint, pfosten1, pfosten2
            else: 
                # Stop‑Kriterium: seit geraumer Zeit kein Tor mehr erkannt
                if timeOut <= 0:
                    self.robotState = STATE_DONE
                    robot.set_wheels(0, 0)
                else: 
                    robot.set_wheels(BASE_SPEED, BASE_SPEED)
                    timeOut -= 1

        elif self.robotState == STATE_GATE_REACHED:
            if timeOut <= 0:
                self.robotState = STATE_DONE
                robot.set_wheels(0, 0)
            else: 
                robot.set_wheels(BASE_SPEED, BASE_SPEED)
                timeOut -= 1
            
        return None
