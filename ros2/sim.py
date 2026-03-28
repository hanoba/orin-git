#!/usr/bin/env python3
# ============================================================================
# TurtleBot‑ähnliche PyGame‑Simulation (korrigiert + ausführlich kommentiert)
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
#  - SPACE: Simulation pausieren/fortsetzen
#
# Skalierung
#  - Alle Längen in Pixeln. Geschwindigkeit in Pixel/s. Zeit in Sekunden.
#  - LiDAR‑Reichweite ist begrenzt, freier Raum wird als MAX_RANGE gemessen.
#
# Hinweise zur Physik
#  - Dies ist KEINE dynamische Simulation (keine Masse/Reibung). Kinematik: v, omega.
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

sys.path.append('../HostSim')
import params


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
LIDAR_NOISE_STD = 0.5                                       # Gauß‑Rauschen (σ) auf Distanzmessung

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


# -------------------------
# LiDAR‑Simulation (NumPy Optimized)
# -------------------------
def cast_lidar(world: World, robot: DiffDriveRobot):
    """
    Berechnet LiDAR-Schnittpunkte vektorisiert mit NumPy.
    FIX: Shape-Mismatch bei 'out' Parameter behoben.
    """

    px = robot.x
    py = robot.y
    lidarX = robot.LIDAR_X_OFFSET
    theta = robot.theta
    
    # 1. Winkelvektor erzeugen
    indices = np.arange(LIDAR_COUNT)
    raw_angles = np.radians(indices - LIDAR_COUNT // 2) 
    global_angles = raw_angles + theta

    # 2. Richtungsvektoren (Shape: [180, 2])
    ray_dirs = np.stack((np.cos(global_angles), np.sin(global_angles)), axis=1)
    
    # Lidarposition (Explizit als Float)
    lx = px + math.cos(theta) * lidarX
    ly = py + math.sin(theta) * lidarX
    
    pos = np.array([lx, ly], dtype=np.float64)

    # Array für Distanzen initialisieren (Shape: [180])
    dists = np.full(LIDAR_COUNT, LIDAR_MAX_RANGE, dtype=np.float64)

    # 3. Schnittpunktberechnung
    for seg in world.segments:
        # Punkte als Float laden
        p1 = np.array([seg.x1, seg.y1], dtype=np.float64)
        p2 = np.array([seg.x2, seg.y2], dtype=np.float64)
        
        seg_vec = p2 - p1
        
        # Denom ist ein Array der Länge 180 (da ray_dirs ein Array ist)
        denom = ray_dirs[:, 0] * seg_vec[1] - ray_dirs[:, 1] * seg_vec[0]

        # Valid Mask
        valid_denom = np.abs(denom) > 1e-9

        # Zähler berechnen
        sx_minus_px = p1[0] - pos[0]
        sy_minus_py = p1[1] - pos[1]
        
        # t_num ist ein SKALAR (hängt nur von Wand & Roboter ab, nicht vom Strahl)
        t_num = sx_minus_px * seg_vec[1] - sy_minus_py * seg_vec[0]
        
        # u_num ist ein ARRAY (hängt von ray_dirs ab)
        u_num = sx_minus_px * ray_dirs[:, 1] - sy_minus_py * ray_dirs[:, 0]

        # --- FIX HIER ---
        # Wir nutzen np.full_like(denom, ...), damit 'out' die Größe 180 hat.
        # Vorher: np.full_like(t_num, ...) -> erzeugte Skalar -> Crash
        
        t = np.divide(t_num, denom, out=np.full_like(denom, np.inf, dtype=np.float64), where=valid_denom)
        u = np.divide(u_num, denom, out=np.full_like(denom, -1.0, dtype=np.float64), where=valid_denom)

        # Logik-Maske
        hit_mask = (t >= 0) & (u >= 0) & (u <= 1) & valid_denom
        
        # Update Distanzen
        dists = np.where(hit_mask, np.minimum(dists, t), dists)

    # 4. Rauschen
    noise = np.random.normal(0, LIDAR_NOISE_STD, LIDAR_COUNT)
    dists = dists + noise
    
    # 5. Clipping
    dists = np.clip(dists, 0, LIDAR_MAX_RANGE)

    # 6. Koordinaten berechnen
    hits_x = lx + dists * ray_dirs[:, 0]
    hits_y = ly + dists * ray_dirs[:, 1]
    
    hits = np.stack((hits_x, hits_y, dists), axis=1)
    
    return hits, raw_angles, dists

def resolve_collisions(robot, world):
    """Vektor-Kollisionsauflösung, die direkt die Eigenschaften der Roboter-Instanz nutzt."""
    
    # === 1. Roboter-Matrizen vorbereiten ===
    pivot = np.array([robot.x, robot.y])
    cos_t, sin_t = np.cos(robot.theta), np.sin(robot.theta)
    
    rot_matrix = np.array([[cos_t, -sin_t], [sin_t,  cos_t]])
    
    # Werte direkt vom Roboter-Objekt abgreifen
    hl = robot.ROBOT_LENGTH / 2.0
    hw = robot.ROBOT_WIDTH / 2.0
    bx = robot.BODY_X_OFFSET
    
    # Lokale Ecken des Gehäuses (exakt wie in InitRobot)
    local_corners = np.array([
        [ hl + bx,  hw], # Vorne Links
        [-hl + bx,  hw], # Hinten Links
        [-hl + bx, -hw], # Hinten Rechts
        [ hl + bx, -hw]  # Vorne Rechts
    ])
    
    corners = (local_corners @ rot_matrix.T) + pivot
    robot_axes = np.array([[cos_t, sin_t], [-sin_t, cos_t]])
    
    # Der geometrische Mittelpunkt des Gehäuses liegt bei (BODY_X_OFFSET, 0)
    geom_center_world = (np.array([bx, 0.0]) @ rot_matrix.T) + pivot

    # === 2. Wände in ein NumPy-Array laden ===
    # (Dieses Array liegt schon fertig in der world-Klasse vor)
    segments = world.segments_array
    
    if len(segments) == 0:
        return

    # === 3. Vektoren aller Wände gleichzeitig berechnen ===
    V = segments[:, 1, :] - segments[:, 0, :] 
    lengths = np.linalg.norm(V, axis=1, keepdims=True)
    
    # Wände mit Länge 0 aussortieren
    valid = (lengths[:, 0] > 0)
    segments, V, lengths = segments[valid], V[valid], lengths[valid]
    
    N = len(segments)
    if N == 0: return

    # Segment-Achsen (Normalen und Parallelen) berechnen
    seg_para = V / lengths
    seg_norm = np.column_stack([-seg_para[:, 1], seg_para[:, 0]])

    # === 4. SAT-Prüfung für ALLE Wände auf einmal ===
    robot_axes_expanded = np.tile(robot_axes, (N, 1, 1)) 
    seg_axes = np.stack([seg_para, seg_norm], axis=1)    
    axes = np.concatenate([robot_axes_expanded, seg_axes], axis=1)

    # Projektionen
    r_projs = np.matmul(axes, corners.T) 
    r_min = np.min(r_projs, axis=2) 
    r_max = np.max(r_projs, axis=2)

    s_projs = np.matmul(axes, segments.transpose(0, 2, 1)) 
    s_min = np.min(s_projs, axis=2)
    s_max = np.max(s_projs, axis=2)

    # Lücken (Gaps) finden
    gaps = (r_max < s_min) | (s_max < r_min)
    intersecting = ~np.any(gaps, axis=1) 

    if not np.any(intersecting):
        return # Keine Kollision

    # === 5. Kollision auflösen ===
    axes_int = axes[intersecting]         
    r_min_int, r_max_int = r_min[intersecting], r_max[intersecting]       
    s_min_int, s_max_int = s_min[intersecting], s_max[intersecting]
    seg_int = segments[intersecting]      

    overlaps = np.minimum(r_max_int - s_min_int, s_max_int - r_min_int) 
    
    min_axis_idx = np.argmin(overlaps, axis=1) 
    min_overlaps = np.min(overlaps, axis=1)    

    K = len(min_overlaps)
    best_axes = axes_int[np.arange(K), min_axis_idx] 

    # Richtung (Push) vom Mittelpunkt weg berechnen
    s_centers = np.mean(seg_int, axis=1) 
    r_center_projs = np.sum(geom_center_world * best_axes, axis=1) 
    s_center_projs = np.sum(s_centers * best_axes, axis=1)         

    push_signs = np.where(r_center_projs < s_center_projs, -1.0, 1.0)
    push_vectors = best_axes * push_signs[:, None] * min_overlaps[:, None] 

    # Bei mehreren Wänden den tiefsten Push nehmen
    deepest_idx = np.argmax(min_overlaps)
    final_push = push_vectors[deepest_idx]

    # === 6. Physik / Position anwenden ===
    robot.x += final_push[0]
    robot.y += final_push[1]
    
    # Leichter Spin beim Aufprall (optional, macht es dynamischer)
    robot.theta = (robot.theta + 0.05) % (2 * np.pi)
    
    print(f"[sim.resolve_collisions] Collision resolved ({K} walls hit)")
    robot.SetSpeed(0.0, 0.0)

#def resolve_collisions(robot: DiffDriveRobot, world: World):
#    """Kollisionsauflösung."""
#    pushed = False
#    for seg in world.segments:
#        # Vektorrechnung (manuell ist hier okay für wenige Segmente)
#        vx, vy = seg.x2 - seg.x1, seg.y2 - seg.y1
#        wx, wy = robot.x - seg.x1, robot.y - seg.y1
#        
#        vlen2 = vx * vx + vy * vy
#        if vlen2 == 0: continue
#        
#        t = max(0.0, min(1.0, (wx * vx + wy * vy) / vlen2))
#        cx = seg.x1 + t * vx
#        cy = seg.y1 + t * vy
#        
#        dx, dy = robot.x - cx, robot.y - cy
#        d2 = dx * dx + dy * dy
#        r = ROBOT_RADIUS
#        
#        if d2 < r * r:
#            d = math.sqrt(max(1e-9, d2))
#            nx, ny = dx / d, dy / d
#            push = (r - d)
#            robot.x += nx * push
#            robot.y += ny * push
#            robot.theta = (robot.theta + 0.05) % math.tau
#            pushed = True
#            
#    if pushed:
#        print("[sim.resolve_collisions] Collision detected")
#        robot.SetSpeed(0.0, 0.0)


def draw_lidar_rays(surf, robot: DiffDriveRobot, hits):
    """Zeichnet Rays. Hits ist jetzt ein NumPy Array (N, 3)."""
    # Lidarposition (Explizit als Float)
    lx = robot.x + math.cos(robot.theta) * robot.LIDAR_X_OFFSET
    ly = robot.y + math.sin(robot.theta) * robot.LIDAR_X_OFFSET

    # Wenn hits ein NumPy Array ist, iterieren wir darüber
    for val in hits:
        hx, hy, d = val[0], val[1], val[2]
        pygame.draw.line(surf, LIDAR_RAY_COLOR, (lx, ly), (hx, hy), 1)
        if d < LIDAR_MAX_RANGE * 0.999:
            pygame.draw.circle(surf, LIDAR_HIT_COLOR, ((hx), (hy)), 2)

class Simulation:
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
        
        pygame.display.set_caption("NumPy Optimized LiDAR Bot")
        self.clock = pygame.time.Clock()
    
        
        self.show_rays = params.SimShowRays   
        self.manual = False     
        self.running = True
        self.debugMode = False
        self.first = True
        self.pause = params.SimPause
        self.traceMode = True
    
        self.fwd = 0.0
        self.turn = 1.0

        self.sim_time_sec = 0.0
        self.font = pygame.font.SysFont(None, 18)

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
        if not self.pause and not self.manual:
            self.robot.SetSpeed(vLinear, omega)
    
    def Step(self):
        self.dt = self.clock.tick(FPS) / 1000.0 
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_SPACE:
                    self.pause = not self.pause
                elif event.key == pygame.K_d:
                    self.debugMode = not self.debugMode
                elif event.key == pygame.K_l:
                    self.show_rays = not self.show_rays
                elif event.key == pygame.K_m:
                    self.manual = not self.manual
                elif event.key == pygame.K_r:
                    self.robot.Reset()
                    self.pause = False
                elif event.key == pygame.K_s:
                    self.traceSurface.fill((0,0,0,0))
                    self.traceMode = not self.traceMode
                elif event.key == pygame.K_t:
                    self.robot.Turn(np.pi/12)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                # right mouse button pressed?
                if event.button == 3:
                    mouse_x, mouse_y = event.pos
                    xm = Xm(mouse_x)
                    ym = Ym(mouse_y)
                    self.robot.SetPose(xm, ym, 0.0)
                
        #self.screen.fill(BG_COLOR)
        self.screen.blit(self.map, (0, 0))

        # --- NumPy LiDAR (10Hz) ---
        if self.numSteps % 3 == 0:
            self.lidar_hits, angles, radius = cast_lidar(self.world, self.robot)
            radius = np.flip(radius*MetersPerPixel)
        else: radius = []

        if self.show_rays:
            draw_lidar_rays(self.screen, self.robot, self.lidar_hits)

        if self.manual:
            keys = pygame.key.get_pressed()
            if keys[pygame.K_UP]: self.fwd += 0.1
            if keys[pygame.K_DOWN]: self.fwd -= 0.1
            if keys[pygame.K_LEFT]: self.turn -= 0.1
            if keys[pygame.K_RIGHT]: self.turn += 0.1
            
            self.robot.SetSpeed(self.fwd, self.turn)

        if self.pause:
            self.fwd = 0.0
            self.turn = 0.0
            self.robot.SetSpeed(0.0, 0.0)

        if self.debugMode:
            if self.first:
                print(f"{self.robot.x=:.2f}  {self.robot.y=:.2f}")
                self.first = False
        else:
            self.first = True
            self.robot.step(self.dt)
            resolve_collisions(self.robot, self.world)

        x, y, theta = self.robot.GetPose()
            
        # Textausgabe mit 3Hz
        if self.numSteps % 10 == 0:
            theta_deg = 360 - np.rad2deg(self.robot.theta)
            if theta_deg > 180: theta_deg -= 360
            txt = (
                f"{self.sim_time_sec:.3f}  FPS: {self.clock.get_fps():3.0f}  "
                f"x={x:6.2f}  y={y:6.2f}  theta={theta_deg:3.0f}°"
            )
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
        
        pygame.display.flip()
        
        self.numSteps += 1
        self.sim_time_sec += self.dt
        
        return x, y, theta, radius
        
    def Quit(self):
        pygame.quit()
    
if __name__ == "__main__":
    sim = Simulation()
    try:
        while sim.running:
            sim.Step()
    except Exception as e:
        print("Error:", e, file=sys.stderr)
        #sim.Quit()
        #raise
    sim.Quit()
    sys.exit()