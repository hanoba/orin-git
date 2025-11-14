#!/usr/bin/env python3

import math
import cmath
import numpy as np
import sys
import ydlidar


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
            ang270 = 270/180*np.pi
            #angles =  np.array([ang270 - p.angle for p in self.scan.points])
            #angles = (angles + np.pi) % (2*np.pi) - np.pi
            #angles =  np.array([(ang270 - p.angle + np.pi) % (2*np.pi) - np.pi for p in self.scan.points])
            # Umrechnung in Roboterkoordinationsystem (x zeigt in Fahrtrichtung
            angles =  np.array([(np.pi - p.angle + np.pi) % (2*np.pi) - np.pi for p in self.scan.points])
            radius  = np.array([p.range for p in self.scan.points])
    
            return angles, radius
            
        else:
            time.sleep(0.05)
            return [], []
            
    def Close(self):
        # === Aufräumen ===
        self.laser.turnOff()
        self.laser.disconnecting()


# LiDAR‑Parameter
#LIDAR_COUNT = 180      #HB 360       # Anzahl der Strahlen (1° Raster)
#LIDAR_MAX_RANGE = 320.0*2            # maximale Messdistanz in Pixeln
#LIDAR_NOISE_STD = 0.5                # Gauß‑Rauschen (σ) auf Distanzmessung

# Roboterkinematik
#ROBOT_RADIUS = 16                    # nur für Zeichnung/Kollision (Kreis)
#self.wheelBase = 2 * 16                   # Radabstand (vereinfacht)
#self.baseSpeed = 70.0                    # Basisfahrgeschwindigkeit [px/s]#
#self.kHeading = 2.2                      # Proportionalgain auf den Richtungsfehler
#MAX_WHEEL_SPEED = 120.0              # Sättigung der Radspeed‑Kommandos [px/s]
##
## Regler‑Gains
#K_DISTANCE = 0.8                     # aktuell nicht genutzt, belassen für Experimente

# Gate‑Detektion
#GATE_MIN_ANGULAR_WIDTH_DEG = 14      # minimale zusammenhängende freie Winkelbreite
#GATE_FREE_RANGE_THRESH = LIDAR_MAX_RANGE * 0.92  # Schwelle ab der ein Strahl „frei“ gilt
#self.gateReachedThreshold = 30               # Schwellwert für Erreichen des Tores
#self.startPointThreshold = 10        # Schwellwert für Erreichen Startpunktes
#self.targetAngleReachedThreshold = 16/180*math.pi       # Schwellwert für Ausrichtung zum Ziel
#GATE_WIDTH_MIN = 100
#GATE_WIDTH_MAX = 140

# Zustände der einfachen Zustandsmaschine
STATE_SEARCH = 0                     # Suchen / „patrouillieren“
STATE_GOTO_START = 1                 # Ausrichten auf Startpunkt und zufahren
STATE_ALIGN_AND_GO = 2               # Ausrichten auf Gate und zufahren
STATE_GATE_REACHED = 3               # Distance to gate less than self.gateReachedThreshold
STATE_DONE = 4                       # hinter dem Tor angehalten
STATE_ERROR = 5                      # Tor seit geraumer Zeit nicht gefunden

def G(x):
    return int(x*360/math.tau)


def wrap_angle(angle):
    """Normiert einen Winkel auf den Bereich [-p, p]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


class Gate():
    def __init__(self, 
        gateWidthMin, 
        gateWidthMax,
        startPointDist, 
        vonRechts, 
        maxWinkel=45, 
        freeRangeDist=6,
        gateIndexMargin=0,                   
        segBasedDetection=False,
        segMinPoints=100,
        segThreshold=0.1
        ):
        # Torbreite 
        self.gateWidthMin = gateWidthMin        # Minimale Breite des Tores
        self.gateWidthMax = gateWidthMax        # Maximale Breite des Tores

        self.startPointDist = startPointDist    # Abstand des Startpunktes vom Tor
        self.vonRechts = vonRechts              # Torsuche von rechts beginnen
        self.maxWinkelRad = maxWinkel/180*np.pi # maximal Öffnungswinkel in Fahrtrichtung
        self.freeRangeDist = freeRangeDist
        self.minInsideGatePoints = 14           # Mindestanzahl der Messwerte im Tor
        self.gateIndexMargin = gateIndexMargin  # zur Rauschunterdrückung am Rand des Tores
        self.segBasedDetection = segBasedDetection
        self.segMinPoints = segMinPoints        # minimum number of points per segments
        self.segThreshold = segThreshold        # maximaler Abstand zur Linie [m]

    def Detect(self, angle, radius, points=None):
        """Erkennt die breiteste „freie“ Winkelmenge im 360°-LiDAR.

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
            print ("Zaun nicht gefunden")     
            return None
        
        # Suche Tor
        cur_len = 0
        i2 = 0
        
        torBreite = 0
        found = False
        gateFreeRangeThresh = self.freeRangeDist
        #print(f"{gateFreeRangeThresh=}")
        for i in range(i0+1,n):
            d = radius[i]
            #print(i,int(d), G(angle[i]), cur_len)  #HB
            if d >= gateFreeRangeThresh:
                if cur_len == 0:
                    i1 = max(i - 1 - self.gateIndexMargin, 0)
                    gateFreeRangeThresh = radius[i1] * 1.5  #+ 0.5
                    #print(f"{gateFreeRangeThresh=}")
                cur_len += 1
            else:
                if cur_len > 0: 
                    i2 = min(i + self.gateIndexMargin, n-1)
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
                        #print(f"{i1=}, {i2=}, {pfosten1=}, {pfosten2=}, {G(phi1)=}°, {G(phi2)=}°")
                        if torBreite <= self.gateWidthMax and torBreite >= self.gateWidthMin:
                            found = True
                            break
                    cur_len = 0
            
        if not found:
            print (f"Tor nicht gefunden {torBreite=}")   
            #print(f"{pfosten1=}, {pfosten2=}")
            return None  # , None
            
        # Winkel zum Mittelpunkt des Tores
        torMitte = (pfosten2 + pfosten1) / 2
        startPoint = torMitte + self.startPointDist * tor / torBreite * (1j if self.vonRechts else -1j)
        #print(f"{pfosten1=}, {pfosten2=}")
        return torMitte, startPoint, pfosten1, pfosten2

    def Preprocessing(self, angles, radius):
        # Winkel-Filter
        mask = (angles >= np.pi/2*0-self.maxWinkelRad) & (angles <= 0*np.pi/2+self.maxWinkelRad)
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


class RobotController():
    def __init__(self, 
                robot, 
                gate, 
                gateReachedThreshold = 0.3,         # Schwellwert für Erreichen des Tores [m]
                startPointThreshold = 0.1,          # Schwellwert für Erreichen Startpunktes
                baseSpeed = 0.2,                    # Basisfahrgeschwindigkeit [m/s]#
                kHeading = 2.2                      # Proportionalgain auf den Richtungsfehler
        ):
        self.robotState = STATE_SEARCH
        self.timeOut = 10
        self.robot = robot                                  # Python-Funktion zum setzen der Radgeschwindigkeit
        self.gate = gate                                    # Tor das gefunden werden soll
        self.gateReachedThreshold = gateReachedThreshold    # Schwellwert für Erreichen des Tores
        self.startPointThreshold = startPointThreshold      # Schwellwert für Erreichen Startpunktes
        self.targetAngleReachedThreshold = 16/180*math.pi   # Schwellwert für Ausrichtung zum Ziel
        self.baseSpeed = baseSpeed                          # Basisfahrgeschwindigkeit [px/s]#
        self.kHeading = kHeading                            # Proportionalgain auf den Richtungsfehler
        print(self.GetState())
        
    def GetState(self):
        return f"{['SEARCH','GOTO_START','ALIGN&GO','GATE_REACHED','DONE','ERROR'][self.robotState]}  "
        
    def SetState(self, state):
        self.robotState = state
        print(self.GetState())
        if state==STATE_SEARCH:         self.robot.SetColor(255, 255, 255)      # weiß
        elif state==STATE_GOTO_START:   self.robot.SetColor(255, 255, 0)        # gelb
        elif state==STATE_ALIGN_AND_GO: self.robot.SetColor(0, 0, 255)          # Blau
        elif state==STATE_GATE_REACHED: self.robot.SetColor(0, 255, 0)          # grün
        elif state==STATE_DONE:         self.robot.SetColor(255, 0, 0)          # rot
        elif state==STATE_ERROR:        self.robot.SetColor(255, 0, 255)        # lila


    def Ready(self):
        return self.robotState == STATE_DONE or self.robotState == STATE_ERROR

    def Reset(self):
        self.SetState(STATE_SEARCH)
    
    def Run(self, angles, radius, dt):
        """Einfache Zustandssteuerung.

        SEARCH:         Dreht sich langsam, bis das Tor erkannt wurde.
        GOTO_START:     Fährt zum Startpunkt
        ALIGN&GO:       Regelt die Ausrichtung auf den Mittelpunkt des Tores und fährt vorwärts.
        GATE_REACHED:   Der Abstand zum Tor ist kleiner als 30cm
        DONE:           Stoppt, kurze Zeit nachdem das Tor erreicht wurde.
        """
        if self.robotState == STATE_DONE:
            self.robot.SetSpeed(0, 0)
            return None

        
        if self.robotState == STATE_SEARCH:
            # roboter drehen um das Panorama zu „scannen“
            self.robot.SetSpeed(0, 2*3.14/4)       # Nur Drehung
            #self.robot.SetSpeed(0, 0)  #HB TEST
            result = self.gate.Detect(angles, radius)
            #sys.exit(0) #HB TEST
            # Beim ersten validen Gate wechseln wir in den Ausrichtungs/Fahrt‑Modus
            if result is not None:
                torMitte, startPoint, pfosten1, pfosten2 = result
                self.targetAngle = cmath.phase(torMitte)  #HB[0]  # Winkel IN ROBOTERKOORDINATEN
                self.SetState(STATE_GOTO_START)
                return torMitte, startPoint, pfosten1, pfosten2

        elif self.robotState == STATE_GOTO_START:
            # Aktualisiere Zielwinkel laufend, falls Gate sich geometrisch verschiebt
            result = self.gate.Detect(angles, radius)
            if result is not None:
                torMitte, startPoint, pfosten1, pfosten2 = result
                if abs(startPoint) > self.startPointThreshold:
                    self.targetAngle = cmath.phase(startPoint)  #HB [0]

                    # Fehler zwischen Blickrichtung und Gate‑Mittelwinkel
                    # Hinweis: Da die LiDAR‑Winkel relativ zu theta erzeugt wurden, ist self.targetAngle
                    # bereits im Roboter‑Frame. 
                    #HB err = wrap_angle(gate[0] - robot.theta) 
                    #err = wrap_angle(self.targetAngle) 
                    err = self.targetAngle
                    #print(f"err={G(err)}°")  #HB
                    omega_cmd = self.kHeading * err
                    HB=0.0
                    if err<0: omega_cmd -= HB
                    else: omega_cmd += HB
                    #if numSteps < 20: print(f"{self.targetAngle=}")   #HB
                    v_cmd = self.baseSpeed*1 #HB  # konstante Vorwärtsfahrt, Stabilität via Heading‑Regelung
                    if abs(self.targetAngle) > self.targetAngleReachedThreshold: v_cmd = 0
                    # Umrechnung in Radspeed‑Kommandos (Differentialfahrwerk)
                    #vl = v_cmd - omega_cmd * (self.wheelBase / 2.0)
                    #vr = v_cmd + omega_cmd * (self.wheelBase / 2.0)
                    self.robot.SetSpeed(v_cmd, omega_cmd)
                    #self.robot.SetSpeed(0, 0)  #HB
                else: 
                    # start point reached
                    self.SetState(STATE_ALIGN_AND_GO)
                    self.robot.SetSpeed(0, 0)
                return torMitte, startPoint, pfosten1, pfosten2
        
        elif self.robotState == STATE_ALIGN_AND_GO:
            # Aktualisiere Zielwinkel laufend, falls Gate sich geometrisch verschiebt
            result = self.gate.Detect(angles, radius)
            if result is not None:
                torMitte, startPoint, pfosten1, pfosten2 = result
                self.targetAngle = cmath.phase(torMitte)  #HB [0]

                # Fehler zwischen Blickrichtung und Gate‑Mittelwinkel
                # Hinweis: Da die LiDAR‑Winkel relativ zu theta erzeugt wurden, ist self.targetAngle
                # bereits im Roboter‑Frame. 
                err = self.targetAngle
                #print(f"err={G(err)}°")  #HB
                omega_cmd = self.kHeading * err
                HB=0.0
                if err<0: omega_cmd -= HB
                else: omega_cmd += HB
                #if numSteps < 20: print(f"{self.targetAngle=}")   #HB
                v_cmd = self.baseSpeed*1 #HB  # konstante Vorwärtsfahrt, Stabilität via Heading‑Regelung
                if abs(self.targetAngle) > self.targetAngleReachedThreshold: v_cmd = 0
                # Umrechnung in Radspeed‑Kommandos (Differentialfahrwerk)
                #vl = v_cmd - omega_cmd * (self.wheelBase / 2.0)
                #vr = v_cmd + omega_cmd * (self.wheelBase / 2.0)
                #self.robot.SetSpeed(vl, vr)
                self.robot.SetSpeed(v_cmd, omega_cmd)
                #self.robot.SetSpeed(0, 0)   #HB
                self.timeOut = 10
                # Stop‑Kriterium: geringer Abstand zum Tor
                if abs(torMitte) <= self.gateReachedThreshold:
                    self.timeOut = 160
                    self.SetState(STATE_GATE_REACHED)
                return torMitte, startPoint, pfosten1, pfosten2
            else: 
                # Stop‑Kriterium: seit geraumer Zeit kein Tor mehr erkannt
                if self.timeOut <= 0:
                    self.SetState(STATE_ERROR)
                    print(self.GetState())
                    self.robot.SetSpeed(0, 0)
                else: 
                    self.robot.SetSpeed(self.baseSpeed, 0)
                    self.timeOut -= 1

        elif self.robotState == STATE_GATE_REACHED:
            if self.timeOut <= 0:
                self.SetState(STATE_DONE)
                self.robot.SetSpeed(0, 0)
            else: 
                self.robot.SetSpeed(self.baseSpeed, 0)
                self.timeOut -= 1
            
        return None
