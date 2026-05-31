#!/usr/bin/env python3
# ============================================================================
# TurtleBot‑ähnliche PyGame‑Visualizer (korrigiert + ausführlich kommentiert)
# ============================================================================
# Zweck
#  - 2‑Rad‑Differentialantrieb (wie TurtleBot: zwei Antriebsräder + freies Stützrad)
#  - 360°‑LiDAR wird simuliert (Strahlenabstand 1°). Kollisionen via Segment‑Ray‑Schnitt.
#  - Ziel: Ein offenes Tor (Lücke in vertikaler Mauer) erkennen und autonom hindurchfahren.
#  - Die Steuerung nutzt einen einfachen P‑Regler auf den Richtungsfehler.
#  - Winkelbezug: 0 rad nach rechts (Mathe‑Standard, +x‑Achse), positive Drehung gegen Uhrzeigersinn.
#  - WICHTIG: LiDAR‑Strahlwinkel sind RELATIV zur Roboterpose (theta). So bleibt das Gate
#             in Roboter‑Koordinaten stabil sichtbar.
#
# Bedienung
#  - ESC: Beenden
#  - L: LiDAR‑Strahlen an/aus
#  - M: manuellen Modus ein/aus (Pfeiltasten steuern v und omega indirekt)
#  - D: Debugmode ein/aus
#  - R: Reset zur Ausgangsposition (Sim-Zeit läuft weiter)
#  - SPACE: Visualizer pausieren/fortsetzen
#
# Skalierung
#  - Alle Längen in Pixeln. Geschwindigkeit in Pixel/s. Zeit in Sekunden.
#  - LiDAR‑Reichweite ist begrenzt, freier Raum wird als MAX_RANGE gemessen.
#
# Hinweise zur Physik
#  - Dies ist KEINE dynamische Visualizer (keine Masse/Reibung). Kinematik: v, omega.
#  - Raddrehgeschwindigkeiten werden hart begrenzt (MAX_WHEEL_SPEED), um numerisch stabil zu bleiben.
# ============================================================================

import math
import cmath
import sys
import numpy as np  # WICHTIG: NumPy importieren
import pygame
from GartenWorld import World, WIN_W, WIN_H, X, Y, V, Xm, Ym, MetersPerPixel
import os

# Setzt den Audio-Treiber auf 'dummy'. 
# Das verhindert, dass Pygame versucht, eine echte Audio-Verbindung aufzubauen.
os.environ["SDL_AUDIODRIVER"] = "dummy"

#sys.path.append('../HostSim')
import params
from params import Udp
from UdpReceive import UdpReceive


# =========================
# Globale Parameter / Konfiguration
# =========================
FPS = 30                             # Simulationsrate (Frames pro Sekunde)
BG_COLOR = (25, 28, 35)              # Hintergrundfarbe
GATE_COLOR = (50, 200, 120)          # Farbe für Tor‑Markierung
ROBOT_COLOR = (80, 160, 255)         # Roboterfarbe
WALL_COLOR = (180, 180, 180)         # Farbe der Wände
POINT_COLOR = (255, 0, 0)            # Farbe für berechnetes Tor und Startpunkt
POINT_RADIUS = 8                     # Punktradius für berechnetes Tor und Startpunkt
LIDAR_RAY_COLOR = (90, 90, 130)      # Linienfarbe der LiDAR‑Strahlen
LIDAR_HIT_COLOR = (220, 220, 80)     # Punktfarbe für LiDAR‑Treffer
TARGET_COLOR = (255, 120, 120)       # Visualisierung der Zielrichtung

# Weltgeometrie
WALL_X = 500                         
GATE_Y1 = 240                        
GATE_Y2 = 360                        

# LiDAR Parameter
LIDAR_COUNT = 2*params.LidarMaxAngle                        # Anzahl der Strahlen (1° Raster)
LIDAR_MAX_RANGE = params.LidarRangeMax / MetersPerPixel     # maximale Messdistanz in Pixeln
LIDAR_NOISE_STD = 0.5*3                                       # Gauß‑Rauschen (σ) auf Distanzmessung

# Roboterkinematik
#ROBOT_RADIUS = 11  # 16                    # nur für Zeichnung/Kollision (Kreis)
#WHEEL_BASE = 2 * ROBOT_RADIUS        # Radabstand (vereinfacht)
MAX_WHEEL_SPEED = 120.0              # Sättigung der Radspeed‑Kommandos [px/s]
#BASE_SPEED = 70.0                    # Basisfahrgeschwindigkeit [px/s]

# Regler Gains
K_HEADING = 2.2                      # Proportionalgain auf den Richtungsfehler
K_DISTANCE = 0.8                     # aktuell nicht genutzt, belassen für Experimente

# Gate Detektion
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


def NormalizeAngle(angle_rad):
    return (angle_rad + math.pi) % math.tau - np.pi

    
class DiffDriveRobot:
    """Kinematik (teilweise optimiert mit NumPy)."""
    def __init__(self, x, y, theta):
        self.v_l = 0.0
        self.v_r = 0.0
        #self.state = STATE_SEARCH
        #self.target_angle = -math.pi
        self.SetPose(x, y, theta)
        self.InitRobot()

    def InitRobot(self):
        # --- Grundgrößen ---
        self.ROBOT_LENGTH =  17*2     # 60
        self.ROBOT_WIDTH =   11*2     # 45
        self.WHEEL_LENGTH =   4*2     # 20
        self.WHEEL_WIDTH =    2     #  8
        self.CASTER_RADIUS =  1     #  6
        
        # --- NEUE KONFIGURATION FÜR POSITIONEN ---
        
        # 1. Gehäuse-Verschiebung (X-Achse): 
        # Wenn > 0, rückt das Gehäuse nach vorne (Räder wirken weiter hinten).
        self.LIDAR_X_OFFSET = int(params.LidarX / MetersPerPixel + 0.5)
        self.BODY_X_OFFSET = self.LIDAR_X_OFFSET - self.ROBOT_LENGTH / 2 #4*2  #15  
        #assert self.BODY_X_OFFSET >= 0
        print(f"{self.BODY_X_OFFSET=}")
        
        # 2. Spurweite (Abstand der Radmitten voneinander):
        # Definiert, wie weit die Räder auseinander stehen.
        self.WHEEL_BASE = 11*2   #50     
        
        # 3. Position der Lenkrolle (X-Achse):
        # Relativ zum Drehpunkt (0,0). Bei 35 liegt sie schön weit vorne im Gehäuse.
        self.CASTER_X = 8   #35       
        
        # --- Lokale Geometrie definieren ---
        # (Zentrum 0,0 ist und bleibt exakt mittig zwischen den Rädern!)
        self.local_points = np.array([
            # Gehäuse (Index 0-3) - verschoben um BODY_X_OFFSET
            [self.ROBOT_LENGTH/2 + self.BODY_X_OFFSET, -self.ROBOT_WIDTH/2],
            [self.ROBOT_LENGTH/2 + self.BODY_X_OFFSET, self.ROBOT_WIDTH/2],
            [-self.ROBOT_LENGTH/2 + self.BODY_X_OFFSET, self.ROBOT_WIDTH/2],
            [-self.ROBOT_LENGTH/2 + self.BODY_X_OFFSET, -self.ROBOT_WIDTH/2],
            
            # Linkes Rad (Zentriert bei X=0, Y=-WHEEL_BASE/2)
            [self.WHEEL_LENGTH/2, -self.WHEEL_BASE/2 - self.WHEEL_WIDTH/2],
            [self.WHEEL_LENGTH/2, -self.WHEEL_BASE/2 + self.WHEEL_WIDTH/2],
            [-self.WHEEL_LENGTH/2, -self.WHEEL_BASE/2 + self.WHEEL_WIDTH/2],
            [-self.WHEEL_LENGTH/2, -self.WHEEL_BASE/2 - self.WHEEL_WIDTH/2],
            
            # Rechtes Rad (Zentriert bei X=0, Y=WHEEL_BASE/2)
            [self.WHEEL_LENGTH/2, self.WHEEL_BASE/2 - self.WHEEL_WIDTH/2],
            [self.WHEEL_LENGTH/2, self.WHEEL_BASE/2 + self.WHEEL_WIDTH/2],
            [-self.WHEEL_LENGTH/2, self.WHEEL_BASE/2 + self.WHEEL_WIDTH/2],
            [-self.WHEEL_LENGTH/2, self.WHEEL_BASE/2 - self.WHEEL_WIDTH/2],
            
            # Lenkrolle Zentrum
            [self.CASTER_X, 0],
            
            # Front (für die Richtungslinie)
            [self.ROBOT_LENGTH/2 + self.BODY_X_OFFSET, 0]
        ])

    def SetPose(self, x, y, theta):
        self.x = self.x0 = X(x)
        self.y = self.y0 = Y(y)
        self.theta = self.theta0 = -theta

    def Turn(self, deltaTheta):
        self.theta = NormalizeAngle(self.theta - deltaTheta)
            
    def GetPose(self):
        return Xm(self.x), Ym(self.y), -self.theta
        
    def Reset(self):
        self.x = self.x0 
        self.y = self.y0 
        self.theta = self.theta0 
        #self.state = STATE_SEARCH
        self.SetSpeed(0, 0)

    def SetSpeed(self, v_cmd, omega_cmd):
        """Setzt Radgeschwindigkeiten mit NumPy-Clipping."""
        v_cmd = V(v_cmd)
        omega_cmd = -omega_cmd
        vl = v_cmd - omega_cmd * (self.WHEEL_BASE / 2.0)
        vr = v_cmd + omega_cmd * (self.WHEEL_BASE / 2.0)
        # NumPy Clip ist effizienter und lesbarer als max(min(...))
        self.v_l = np.clip(vl, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
        self.v_r = np.clip(vr, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)

    def step(self, dt):
        """Update der Pose."""
        v = (self.v_r + self.v_l) * 0.5
        omega = (self.v_r - self.v_l) / self.WHEEL_BASE
        
        # Winkel normieren auf [0, 2π) mit math.tau
        self.theta = (self.theta + omega * dt + math.tau) % math.tau
        
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

    def draw(self, surf):
        # 1. Rotationsmatrix
        c = np.cos(self.theta)
        s = np.sin(self.theta)
        
        R_T = np.array([
            [c, s],
            [-s, c]
        ])
        
        # 2. Vektorisierte Rotation und Translation
        global_points = (self.local_points @ R_T) + np.array([self.x, self.y])
        
        # 3. Zeichnen
        #ROBOT_COLOR = (0, 255, 0)       # Grün
        WHEEL_COLOR = (150, 150, 150)   # Hellgrau, damit es auf dunklem Grund auffällt
        
        pygame.draw.polygon(surf, ROBOT_COLOR, global_points[0:4], 1)
        pygame.draw.polygon(surf, WHEEL_COLOR, global_points[4:8], 0)
        pygame.draw.polygon(surf, WHEEL_COLOR, global_points[8:12], 0)
        #pygame.draw.circle(surf, WHEEL_COLOR, global_points[12], self.CASTER_RADIUS)
        pygame.draw.line(surf, ROBOT_COLOR, (self.x, self.y), global_points[13], 1)

    #def OLDdraw(self, surf):
    #    pygame.draw.circle(surf, ROBOT_COLOR, ((self.x), (self.y)), ROBOT_RADIUS, 2)
    #    hx = self.x + math.cos(self.theta) * ROBOT_RADIUS
    #    hy = self.y + math.sin(self.theta) * ROBOT_RADIUS
    #    pygame.draw.line(surf, ROBOT_COLOR, (self.x, self.y), (hx, hy), 2)

    def drawPoint(self, surf, point):
        # cmath wird hier beibehalten, da 'point' vermutlich complex ist (aus RobotLib2)
        p = point * cmath.exp(1j * self.theta)
        x = (self.x + p.real)
        y = (self.y + p.imag)
        pygame.draw.circle(surf, POINT_COLOR, (x, y), POINT_RADIUS, 2)

    def SetColor(self, r, g, b):
        pass


def G(x):
    return x * 360 / math.tau


class Visualizer:
    def __init__(self):
        self.numSteps = 0
        self.robot = DiffDriveRobot(
            x=params.RobotInitX,
            y=params.RobotInitY,
            theta=params.RobotInitTheta)
        self.robot.SetSpeed(0.0,0.0)
    
        pygame.init()
        
        # Falls Fullscreen gewünscht ist:
        # screen = pygame.display.set_mode((screen_width, screen_height), pygame.FULLSCREEN)
        # Für Fenstermodus (besser zum Debuggen):
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        self.map = pygame.Surface((WIN_W, WIN_H))
        self.world = World()
        self.DrawWorld(self.world)
        #self.world.draw(self.map)

        # Die Spur-Ebene (Surface) erstellen
        # Perflags=pygame.SRCALPHA macht sie transparent
        self.traceSurface = pygame.Surface((WIN_W, WIN_H), pygame.SRCALPHA)
        
        pygame.display.set_caption("E-Karren Visualizer")
        self.clock = pygame.time.Clock()
    
        
        self.running = True
        self.debugMode = False
        self.first = True
        self.traceMode = True
    
        self.fwd = 0.0
        self.turn = 1.0

        self.sim_time_sec = 0.0
        self.font = pygame.font.SysFont(None, 18)
        
        self.udp_text = "Hello"
        self.InitMarkers()
        self.hideRedLines = False
        self.hideGreenLines = False
        self.hideBlueLines = False

    def DrawWorld(self, world):
        surf = self.map
        self.DrawGrid(surf)
        for s in world.segments:
            pygame.draw.line(surf, WALL_COLOR, (s.x1, s.y1), (s.x2, s.y2), 2)
        #    s.draw(surf, WALL_COLOR)
        # Tor Visualisierung
        # pygame.draw.line(surf, GATE_COLOR, (WALL_X, GATE_Y1), (WALL_X, GATE_Y2), 3)

    def DrawGrid(self, surf):
        g = 50
        gridColor = (g, g, g)
        ymin = Y(-15)
        ymax = Y(12)
        for x in range(-25,25):
            xp = X(x)
            thickness = 3 if x % 10 == 0 else 1
            pygame.draw.line(surf, gridColor, (xp, ymin), (xp, ymax), thickness)

        xmin = X(-25)
        xmax = X(25)              
        for y in range(-25,25):
            yp = Y(y)
            thickness = 3 if y % 10 == 0 else 1
            pygame.draw.line(surf, gridColor, (xmin, yp), (xmax, yp), thickness)

        # Nullpunkt mit Kreuz markieren
        def P(x,y):
            return (X(x), Y(y))
        np = 1.0
        pygame.draw.line(surf, (255, 0, 0), P(0, 0), P(np, 0), 3)
        pygame.draw.line(surf, (0, 255, 0), P(0, 0), P(0, np), 3)

    def SetRobotSpeed(self, vLinear, omega):
        self.robot.SetSpeed(vLinear, omega)
    
    def Step(self):
        self.dt = self.clock.tick(FPS) / 1000.0 
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_d:
                    self.debugMode = not self.debugMode
                elif event.key == pygame.K_s:
                    self.traceSurface.fill((0,0,0,0))
                    self.traceMode = not self.traceMode
                elif event.key == pygame.K_r:
                    self.hideRedLines = not self.hideRedLines
                elif event.key == pygame.K_g:
                    self.hideGreenLines = not self.hideGreenLines
                elif event.key == pygame.K_b:
                    self.hideBlueLines = not self.hideBlueLines
                
        #self.screen.fill(BG_COLOR)
        self.screen.blit(self.map, (0, 0))

        if self.debugMode:
            if self.first:
                print(f"{self.robot.x=:.2f}  {self.robot.y=:.2f}")
                self.first = False
        else:
            self.first = True
            ###self.robot.step(self.dt)
            ###resolve_collisions(self.robot, self.world)

        x, y, theta = self.robot.GetPose()
            
        # Textausgabe mit 3Hz
        if self.numSteps % 10 == 0:
            theta_deg = 360 - np.rad2deg(self.robot.theta)
            if theta_deg > 180: theta_deg -= 360
            txt = f"theta={theta_deg:3.0f}°   {self.udp_text}"
            self.bitBlock = self.font.render(txt, True, (220, 220, 220))

        if self.traceMode: 
            # Spur auf die EXTRA Surface zeichnen (nicht den Screen!)
            # Hier nutzen wir einen kleinen Kreis für die Spur
            #pygame.draw.circle(self.traceSurface, (0, 255, 100, 150), (self.robot.x, self.robot.y), 2)
            # Syntax: screen.set_at((x-Koordinate, y-Koordinate), (R, G, B))
            self.traceSurface.set_at((int(self.robot.x), int(self.robot.y)), (0, 0, 255))  # Zeichnet einen blauen Pixel
            # Die Spur-Ebene auf den Hauptbildschirm "blitten" (überlagern)
            self.screen.blit(self.traceSurface, (0, 0))
        
        self.robot.draw(self.screen)
                
        # HUD zeichnen (mit Flip für Koordinatensystem, falls nötig)
        # Hier normal zeichnen:
        self.screen.blit(self.bitBlock, (10, 10))

        # UDP-Kommandos einlesen und Marker zeichnen
        self.HandleUdpCommands()
        self.DrawMarkerLines()
        
        pygame.display.flip()
        
        self.numSteps += 1
        self.sim_time_sec += self.dt
        
    def HandleUdpCommands(self):
        # Alle vorhandenen UDP-Kommandos empfangen und dekodieren
        while True:
            udp_command = UdpReceive()
            if udp_command is None:
                break;
                
            udp_header, udp_data = udp_command
            if udp_header == Udp.POSE:
                meter = 0.01
                xu = udp_data[0]*meter
                yu = udp_data[1]*meter
                tu = math.radians(udp_data[2])
                self.robot.SetPose(xu,yu,tu)
                #print(xu,yu,tu)    
            elif udp_header == Udp.MARKER_LINES:
                # Kommando-Parameter:
                # frameType, endPointColor, sx1, sy1, ex1, ey1, ... sxN, syN, exN, eyN, lineColor1, ... lineColorN
                frameType = udp_data[0]
                self.markerEndPointColor = self.Color(udp_data[1])
                data_len = len(udp_data)
                num_lines = (data_len - 2) // 5
                assert num_lines > 0
                assert num_lines*5 == data_len-2
                n = 2 + 4*num_lines
                #print(num_lines)
                self.CreateMarkerLines(udp_data[2:n], frameType)
                #self.CreateMarkerLineColors(udp_data[n:])
                self.markerLineColors = udp_data[n:]
            elif udp_header == Udp.MARKER_DELETE:
                self.InitMarkers()
            elif udp_header == Udp.TEXT:
                self.udp_text = udp_data
        
    def Color(self, c):
        if c==Udp.RED: return (200, 0, 0)
        elif c==Udp.GREEN: return (0, 200, 0)
        elif c==Udp.BLUE: return (0, 0, 200)
        assert c==Udp.WHITE
        return (200, 200, 200)

    def InitMarkers(self):
        self.markerLinePoints = None
        self.markerEndPointColor = self.Color(Udp.WHITE)
        self.markerLineColors = []
    
    def CreateMarkerLineColors(self, colors):
        self.markerLineColors = []
        for c in colors:
            self.markerLineColors.append(self.Color(c))

    def CreateMarkerLines(self, points_list, frameType):
        x, y, theta_rad = self.robot.GetPose()

        # Position des Roboters als NumPy-Arrays definieren
        robotPos = np.array([x, y])        

        # Offset des Lidar-Sensors als NumPy-Arrays definieren
        lidarOffset = np.array([params.LidarX, 0])        

        # Flache Liste in ein Array mit [x, y] Paaren umwandeln (N Zeilen, 2 Spalten) und umrechnen von cm in meter
        meter = 0.01
        points = np.array(points_list).reshape(-1, 2) * meter + lidarOffset
        
        if frameType == Udp.FRAME_MAP:
            self.markerLinePoints = points
            return
        
        # Rotationsmatrix erstellen
        c, s = np.cos(theta_rad), np.sin(theta_rad)
        # Da wir Zeilenvektoren haben, nutzen wir die transponierte Rotationsmatrix
        rot_matrix = np.array([
            [ c, s],
            [-s, c]
        ])
        
        # Alle Punkte auf einmal rotieren (Matrixmultiplikation) und Offet addieren
        self.markerLinePoints = points.dot(rot_matrix) + robotPos
        
    def DrawMarkerLines(self):
        if self.markerLinePoints is None:
            return
            
        # In 2er-Schritten durch das Array gehen, um Punktepaare zu bilden
        for i in range(0, len(self.markerLinePoints), 2):
            # Sicherheitscheck, falls das Array eine ungerade Anzahl an Punkten hätte
            if i + 1 < len(self.markerLinePoints):
                # Logik um Linien auszublenden
                if self.markerLineColors[i // 2] == Udp.RED and self.hideRedLines:
                    continue
                if self.markerLineColors[i // 2] == Udp.GREEN and self.hideGreenLines:
                    continue
                if self.markerLineColors[i // 2] == Udp.BLUE and self.hideBlueLines:
                    continue

                # Startpunkt P(2n) und Endpunkt P(2n+1) extrahieren
                p_start = self.markerLinePoints[i]
                p_end = self.markerLinePoints[i+1]
                
                # Startpunkt in Pygame-Pixel umrechnen
                start_x = X(p_start[0])
                start_y = Y(p_start[1])
                
                # Endpunkt in Pygame-Pixel umrechnen
                end_x = X(p_end[0])
                end_y = Y(p_end[1])
                
                #print(start_x, start_y)

                # Linie zwischen Start- und Endpunkt zeichnen
                # Parameter: (Oberfläche, Farbe, Start-Tupel, End-Tupel, Linienbreite)
                color = self.Color(self.markerLineColors[i // 2])
                pygame.draw.line(self.screen, color, (start_x, start_y), (end_x, end_y), 3)
                
                # Optional: Kleine Kreise auf die Eckpunkte setzen, um sie zu betonen
                pygame.draw.circle(self.screen, self.markerEndPointColor, (start_x, start_y), 4)
                pygame.draw.circle(self.screen, self.markerEndPointColor, (end_x, end_y), 4)            

    def Quit(self):
        pygame.quit()
    
if __name__ == "__main__":
    viz = Visualizer()
    try:
        while viz.running:
            viz.Step()
    except Exception as e:
        print("Error:", e, file=sys.stderr)
    viz.Quit()
    sys.exit()