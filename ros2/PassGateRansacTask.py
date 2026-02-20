import numpy as np
import math
import cmath
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import Ransac
import params
from params import TaskState

def G(x):
    return np.rad2deg(x)

def NormalizeAngle(angle_rad):
    return (angle_rad + math.pi) % math.tau - np.pi

class PassGateRansacTask:
    def Init(self, node, params, retvals=None):
        theta_deg, self.startPointDist, self.min_len, self.max_len = params
        self.StateSearch = 0        # Dreht sich langsam, bis das Tor erkannt wurde.
        self.StateGotoStart = 1     # Fährt zum Startpunkt
        self.StateAlignAndGo = 2    # Regelt die Ausrichtung auf den Mittelpunkt des Tores und fährt vorwärts.
        self.StateGateReached = 3   # Der Abstand zum Tor ist kleiner als 30cm
        self.StateDone = 4          # Stoppt, kurze Zeit nachdem das Tor erreicht wurde.
        self.StateError = 5         # Error detected    
        
        self.followRight = True
        self.vLinear = 0.5  #0.5
        self.K_lat = 0.5 # 0.5          # 0.3       # lateral error (Abstand)
        self.K_head = 0.8 # 0.7          # 0.5      # heading error (Winkel)
        self.node = node
        self.wantedTheta = np.deg2rad(theta_deg)
        self.node.SetWantedTheta(self.wantedTheta)
        self.min_angle = 85
        self.max_angle = 95

        self.gateReachedTimeOut = 80
        self.noGateTimeOut = 10
        self.timeOutCnt = self.noGateTimeOut
        self.gateReachedThreshold = 0.3                     # Schwellwert für Erreichen des Tores
        self.startPointThreshold = 0.1                      # Schwellwert für Erreichen Startpunktes
        self.startPointDist = 2.0                           # Abstand des Startpunktes vom Tor
        self.targetAngleReachedThreshold = np.deg2rad(8)    # Schwellwert für Ausrichtung zum Ziel
        self.baseSpeed = 0.2                                # Basisfahrgeschwindigkeit [m/s] (Tordurchfahrt)
        self.fastSpeed = 0.5                                # Schnelle Fahrgeschwindigkeit [m/s] (für Fahrten zum Startpunkt)
        self.kHeading = 0.8                                 # Proportionalgain auf den Richtungsfehler
        self.SetState(self.StateSearch)
        
    def GetState(self):
        return f"{['SEARCH','GOTO_START','ALIGN&GO','GATE_REACHED','DONE','ERROR'][self.state]}  "
        
    def SetState(self, newState):
        self.state = newState
        self.node.RvizPrint("PassGateRansacTask." + self.GetState())

    def Ready(self):
        return self.state == self.StateDone or self.state == self.StateError

    def Reset(self):
        self.SetState(self.StateSearch)
    
    def Step(self, scan_msg):
        """ Einfache Zustandssteuerung """
        if self.state == self.StateSearch:
            if self.node.wantedThetaReached:
                self.state = self.StateGotoStart
                self.node.ResetDirection()
                
        elif self.state == self.StateGotoStart:
            # Aktualisiere Zielwinkel laufend, falls Gate sich geometrisch verschiebt
            result = self.GateDetector(scan_msg)
            if result is not None:
                torMitte, startPoint, pfosten1, pfosten2 = result
                if abs(startPoint) > self.startPointThreshold:
                    self.targetAngle = cmath.phase(startPoint)
                    err = self.targetAngle
                    omega_cmd = self.kHeading * err
                    v_cmd = self.fastSpeed  # konstante Vorwärtsfahrt, Stabilität via Heading‑Regelung
                    if abs(self.targetAngle) > self.targetAngleReachedThreshold: v_cmd = 0
                    self.node.SetVelocities(omega_cmd, v_cmd)
                else: 
                    # start point reached
                    self.SetState(self.StateAlignAndGo)
                    self.node.SetVelocities(0, 0)
            else: self.node.SetVelocities(0, 0)
        
        elif self.state == self.StateAlignAndGo:
            # Aktualisiere Zielwinkel laufend, falls Gate sich geometrisch verschiebt
            result = self.GateDetector(scan_msg)
            if result is not None:
                torMitte, startPoint, pfosten1, pfosten2 = result
                self.targetAngle = cmath.phase(torMitte)  #HB [0]

                err = self.targetAngle
                print(f"err={G(err)}°")  #HB
                omega_cmd = self.kHeading * err
                v_cmd = self.baseSpeed  # konstante Vorwärtsfahrt, Stabilität via Heading‑Regelung
                if abs(self.targetAngle) > self.targetAngleReachedThreshold: v_cmd = 0
                self.node.SetVelocities(omega_cmd, v_cmd)
                self.timeOutCnt = self.noGateTimeOut
                # Stop‑Kriterium: geringer Abstand zum Tor
                if abs(torMitte) <= self.gateReachedThreshold:
                    self.timeOutCnt = self.gateReachedTimeOut
                    self.SetState(self.StateGateReached)
            else: 
                # Stop‑Kriterium: seit geraumer Zeit kein Tor mehr erkannt
                self.node.SetVelocities(0, 0)
                if self.timeOutCnt <= 0:
                    self.SetState(self.StateError)
                else: 
                    self.timeOutCnt -= 1

        elif self.state == self.StateGateReached:
            if self.timeOutCnt <= 0:
                self.SetState(self.StateDone)
                self.node.SetVelocities(0, 0)
            else: 
                self.node.SetVelocities(0, self.baseSpeed)
                self.timeOutCnt -= 1

        elif self.state == self.StateDone:
            self.node.SetVelocities(0, 0)
            return TaskState.Ready, None

        elif self.state == self.StateError:
            self.node.SetVelocities(0, 0)
            return TaskState.Error, None
            
        return TaskState.Running, None








    def RotateWalls(self, alpha, walls):
        # walls ins Weltkoordinatensystem drehen
        # walls hat die Form (n, 2, 2)
        # n = Anzahl der Segmente
        # 2 = Start- und Endpunkt pro Segment
        # 2 = x- und y-Koordinate
        # Deine 2x2 Drehmatrix
        c, s = np.cos(alpha), np.sin(alpha)
        M = np.array([[c, -s], 
                      [s,  c]])

        # Die Magie von NumPy:
        # (n, 2, 2) @ (2, 2) -> ergibt (n, 2, 2)
        walls = walls @ M
        return walls

    def FilterWalls(self, walls):
        """
        Filtert Linien basierend auf Länge und Winkel (Richtung ignoriert).
        Sortiert die gefilterten Linien nach Länge (absteigend).
        
        Parameters:
        walls: np.array der Shape (n, 2, 2) -> n Linien, 2 Punkte (Start/Ende), 2 Koord (x, y)
        min_len, max_len: Bereich der Länge
        min_angle, max_angle: Bereich des Winkels in Grad (0 bis 180)
        """
        
        # Differenzvektoren berechnen (dx, dy)
        # Shape: (n, 2)
        diff = walls[:, 1, :] - walls[:, 0, :]
        dx = diff[:, 0]
        dy = diff[:, 1]
        
        # Länge berechnen (Pythagoras)
        lengths = np.sqrt(dx**2 + dy**2)

        # 3. Winkel berechnen (Richtung ignorieren)
        # Wir nutzen arctan2, schränken aber die Richtung auf 0-180 Grad ein.
        # Trick: Wenn dy negativ ist, drehen wir den Vektor um 180 Grad um (Punkt A/B Tausch).
        angles_rad = np.arctan2(dy, dx)
        angles_deg = np.degrees(angles_rad) % 180  # Normiert auf [0, 180)

        # x-Koordinaten berechnen
        x0 = walls[:, 0, 0]
        x1 = walls[:, 1, 0]

        # 4. Maske erstellen (Boolean Indexing)
        length_mask = (lengths >= self.min_len) & (lengths <= self.max_len)
        angle_mask = (angles_deg >= self.min_angle) & (angles_deg <= self.max_angle)
        x_mask = (x0 > 0.0) & (x1 > 0.0) & (x0 < 4.0) & (x1 < 4.0)
        
        final_mask = length_mask & angle_mask & x_mask
        walls = walls[final_mask]

        # nach Länge sortieren (absteigend)
        lengths = lengths[final_mask]
        indices = np.argsort(lengths)[::-1]
        walls = walls[indices]
        
        # Gefilterte und sortierte Submatrix zurückgeben
        return walls

    def FindTor(self, walls, w):
        numWalls, _, _ = walls.shape
        if numWalls < 2:
            return None
           
        ymin = min(walls[0,0,1], walls[0,1,1])
        ymax = max(walls[0,0,1], walls[0,1,1])
        for i in range(1,numWalls):
            if walls[i,0,1] < ymin and walls[i,1,1] < ymin:
                pfosten1 = complex(w[i,0,0], w[i,0,1]) if walls[i,0,1] > walls[i,1,1] else complex(w[i,1,0], w[i,1,1])
                pfosten2 = complex(w[0,0,0], w[0,0,1]) if walls[0,0,1] < walls[0,1,1] else complex(w[0,1,0], w[0,1,1])
                return pfosten1, pfosten2
            elif walls[i,0,1] > ymax and walls[i,1,1] > ymax:
                pfosten1 = complex(w[0,0,0], w[0,0,1]) if walls[0,0,1] > walls[0,1,1] else complex(w[0,1,0], w[0,1,1])
                pfosten2 = complex(w[i,0,0], w[i,0,1]) if walls[i,0,1] < walls[i,1,1] else complex(w[i,1,0], w[i,1,1])
                return pfosten1, pfosten2
        return None
    
    def Walldetector(self, msg):
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        rangesMask = np.isfinite(ranges) & (ranges > params.LidarRangeMin + 0.01) & (ranges < params.LidarRangeMax - 0.1)
        #maxAngle = np.pi * 0.5
        #anglesMask = np.abs(angles) < maxAngle
        finalMask = rangesMask # & anglesMask
        points = np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))[finalMask]
        allDetectedWalls = Ransac.LineDetection(points)
        return allDetectedWalls

    def GateDetector(self, scan_msg):
        #s = -1.0 if self.followRight else 1.0
        all_detected_walls = self.Walldetector(scan_msg) 
        walls1 = np.array(all_detected_walls)
        #self.PublishMarkers(walls1, [])
        #print(f"{walls1.shape=}")
        alpha = self.wantedTheta - self.node.theta
        rotatedWalls = self.RotateWalls(alpha, walls1)
        rotatedWalls = self.FilterWalls(rotatedWalls)
        #print(f"{rotatedWalls.shape=}")
        walls = self.RotateWalls(-alpha, rotatedWalls)
        retvals = self.FindTor(rotatedWalls, walls)
        if retvals is not None:
            pfosten1, pfosten2 = retvals
            torMitte = (pfosten2 + pfosten1) / 2
            tor = pfosten2 - pfosten1
            torBreite = abs(tor)
            startPoint = torMitte + self.startPointDist * tor / torBreite * 1j  #(1j if self.vonRechts else -1j)
            torPunkte = [pfosten1, pfosten2, torMitte, startPoint]
            self.PublishMarkers(walls, torPunkte)
            return torMitte, startPoint, pfosten1, pfosten2
        return None

    def PublishMarkers(self, walls, punkte):
        pub = self.node.marker_pub
        markers = MarkerArray()
        frame_id = "lidar"
        z_height = +0.1
        
        # 1. Ein Marker für ALLE Linien
        m_lines = Marker()
        m_lines.header.frame_id = frame_id
        m_lines.header.stamp.sec = 0
        m_lines.header.stamp.nanosec = 0
        m_lines.ns = "walls_lines"
        m_lines.id = 0
        m_lines.type = Marker.LINE_LIST 
        m_lines.action = Marker.ADD
        m_lines.scale.x = 0.15
        m_lines.color.g = 1.0; m_lines.color.a = 0.8
        m_lines.pose.orientation.w = 1.0

        # 2. Ein Marker für ALLE Endpunkte (Sphären)
        m_spheres = Marker()
        m_spheres.header = m_lines.header
        m_spheres.ns = "walls_endpoints"
        m_spheres.id = 0
        m_spheres.type = Marker.SPHERE_LIST # <--- EXTREM EFFIZIENT
        m_spheres.action = Marker.ADD
        m_spheres.scale.x = m_spheres.scale.y =  m_spheres.scale.z = 0.25*2
        m_spheres.color.b = 1.0; m_spheres.color.a = 1.0

        for start, end in walls:
            p_start = Point(x=float(start[0]), y=float(start[1]), z=z_height)
            p_end = Point(x=float(end[0]), y=float(end[1]), z=z_height)

            wall_color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8) # if isDetectedWallValid[i] else ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)        
            
            # Punkte zur Linienliste hinzufügen
            m_lines.points.append(p_start)
            m_lines.points.append(p_end)

            # Farbe zweimal hinzufügen (für beide Enden des Segments)
            m_lines.colors.append(wall_color)
            m_lines.colors.append(wall_color)
            
        for p in punkte:
            # Punkt zur Sphärenliste hinzufügen
            sphere = Point(x=float(p.real), y=float(p.imag), z=z_height)
            m_spheres.points.append(sphere)

        markers.markers.append(m_lines)
        markers.markers.append(m_spheres)
        
        pub.publish(markers)
