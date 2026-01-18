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
#  - R: LiDAR‑Strahlen an/aus
#  - M: manuellen Modus ein/aus (Pfeiltasten steuern v und omega indirekt)
#  - D: Debugmode ein/aus
#  - SPACE: Reset an Ausgangsposition vor dem Tor
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
from dataclasses import dataclass
import pygame
from RobotLib2 import Gate, RobotController

numSteps = 0

# =========================
# Globale Parameter / Konfiguration
# =========================
WIN_W, WIN_H = 900, 600              # Fenstergröße in Pixeln
FPS = 30                             # Simulationsrate (Frames pro Sekunde)
BG_COLOR = (25, 28, 35)              # Hintergrundfarbe
WALL_COLOR = (180, 180, 180)         # Farbe der Wände
GATE_COLOR = (50, 200, 120)          # Farbe für Tor‑Markierung
ROBOT_COLOR = (80, 160, 255)         # Roboterfarbe
POINT_COLOR = (255, 0, 0)            # Farbe für berechnetes Tor und Startpunkt
POINT_RADIUS = 8                     # Punktradius für berechnetes Tor und Startpunkt
LIDAR_RAY_COLOR = (90, 90, 130)      # Linienfarbe der LiDAR‑Strahlen
LIDAR_HIT_COLOR = (220, 220, 80)     # Punktfarbe für LiDAR‑Treffer
TARGET_COLOR = (255, 120, 120)       # Visualisierung der Zielrichtung

# Weltgeometrie
BORDER_MARGIN = 40                   
WALL_X = 500                         
GATE_Y1 = 240                        
GATE_Y2 = 360                        

# LiDAR Parameter
LIDAR_COUNT = 240                    # Anzahl der Strahlen (1° Raster)
LIDAR_MAX_RANGE = 320.0*3            # maximale Messdistanz in Pixeln
LIDAR_NOISE_STD = 0.5                # Gauß‑Rauschen (σ) auf Distanzmessung

# Roboterkinematik
ROBOT_RADIUS = 16                    # nur für Zeichnung/Kollision (Kreis)
WHEEL_BASE = 2 * ROBOT_RADIUS        # Radabstand (vereinfacht)
MAX_WHEEL_SPEED = 120.0              # Sättigung der Radspeed‑Kommandos [px/s]
BASE_SPEED = 70.0                    # Basisfahrgeschwindigkeit [px/s]

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


@dataclass
class Segment:
    """Liniensegment (Wand/Begrenzung) in Weltkoordinaten."""
    x1: float
    y1: float
    x2: float
    y2: float

    def draw(self, surf, color):
        pygame.draw.line(surf, color, (self.x1, self.y1), (self.x2, self.y2), 2)
    
    # Neu für NumPy: Segment als Vektor zurückgeben
    def to_numpy(self):
        return np.array([self.x1, self.y1]), np.array([self.x2, self.y2])


class World:
    """Enthält alle kollidierenden Liniensegmente und zeichnet die Szene."""
    def __init__(self):
        self.segments = []
        # Außenrahmen erzeugen
        x0, y0 = BORDER_MARGIN, BORDER_MARGIN
        x1, y1 = WIN_W - BORDER_MARGIN, WIN_H - BORDER_MARGIN
        x1_wall = WALL_X      
        
        # Außenrahmen
        self.add_rect_border(x0, y0, x1_wall, y1)
        # Tor Segmente
        self.segments.append(Segment(WALL_X, y0, WALL_X, GATE_Y1))
        self.segments.append(Segment(WALL_X, GATE_Y2, WALL_X, y1))

    def add_rect_border(self, x0, y0, x1, y1):
        self.segments += [
            Segment(x0, y0, x1, y0),  # oben
            # Segment(x1, y0, x1, y1),  # rechts (offen lassen?)
            Segment(x1, y1, x0, y1),  # unten
            Segment(x0, y1, x0, y0),  # links
        ]

    def draw(self, surf):
        for s in self.segments:
            s.draw(surf, WALL_COLOR)
        # Tor Visualisierung
        # pygame.draw.line(surf, GATE_COLOR, (WALL_X, GATE_Y1), (WALL_X, GATE_Y2), 3)


# -------------------------
# LiDAR‑Simulation (NumPy Optimized)
# -------------------------
def cast_lidar(world: World, px, py, theta=0.0):
    """
    Berechnet LiDAR-Schnittpunkte vektorisiert mit NumPy.
    FIX: Shape-Mismatch bei 'out' Parameter behoben.
    """
    
    # 1. Winkelvektor erzeugen
    indices = np.arange(LIDAR_COUNT)
    raw_angles = np.radians(indices - LIDAR_COUNT // 2) 
    global_angles = raw_angles + theta

    # 2. Richtungsvektoren (Shape: [180, 2])
    ray_dirs = np.stack((np.cos(global_angles), np.sin(global_angles)), axis=1)
    
    # Roboterposition (Explizit als Float)
    pos = np.array([px, py], dtype=np.float64)

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
    hits_x = px + dists * ray_dirs[:, 0]
    hits_y = py + dists * ray_dirs[:, 1]
    
    hits = np.stack((hits_x, hits_y, dists), axis=1)
    
    return hits, raw_angles, dists
class DiffDriveRobot:
    """Kinematik (teilweise optimiert mit NumPy)."""
    def __init__(self, x, y, theta):
        self.x = self.x0 = x
        self.y = self.y0 = y
        self.theta = self.theta0 = theta
        self.v_l = 0.0
        self.v_r = 0.0
        self.state = STATE_SEARCH
        self.target_angle = -math.pi

    def Reset(self):
        self.x = self.x0 
        self.y = self.y0 
        self.theta = self.theta0 
        self.state = STATE_SEARCH
        self.SetSpeed(0, 0)

    def SetSpeed(self, v_cmd, omega_cmd):
        """Setzt Radgeschwindigkeiten mit NumPy-Clipping."""
        vl = v_cmd - omega_cmd * (WHEEL_BASE / 2.0)
        vr = v_cmd + omega_cmd * (WHEEL_BASE / 2.0)
        # NumPy Clip ist effizienter und lesbarer als max(min(...))
        self.v_l = np.clip(vl, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
        self.v_r = np.clip(vr, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)

    def step(self, dt):
        """Update der Pose."""
        v = (self.v_r + self.v_l) * 0.5
        omega = (self.v_r - self.v_l) / WHEEL_BASE
        
        # Winkel normieren auf [0, 2π) mit math.tau
        self.theta = (self.theta + omega * dt + math.tau) % math.tau
        
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

    def draw(self, surf):
        pygame.draw.circle(surf, ROBOT_COLOR, (int(self.x), int(self.y)), ROBOT_RADIUS, 2)
        hx = self.x + math.cos(self.theta) * ROBOT_RADIUS
        hy = self.y + math.sin(self.theta) * ROBOT_RADIUS
        pygame.draw.line(surf, ROBOT_COLOR, (self.x, self.y), (hx, hy), 2)

    def drawPoint(self, surf, point):
        # cmath wird hier beibehalten, da 'point' vermutlich complex ist (aus RobotLib2)
        p = point * cmath.exp(1j * self.theta)
        x = int(self.x + p.real)
        y = int(self.y + p.imag)
        pygame.draw.circle(surf, POINT_COLOR, (x, y), POINT_RADIUS, 2)

    def SetColor(self, r, g, b):
        pass


def G(x):
    return x * 360 / math.tau


def resolve_collisions(robot: DiffDriveRobot, world: World):
    """Kollisionsauflösung."""
    pushed = False
    for seg in world.segments:
        # Vektorrechnung (manuell ist hier okay für wenige Segmente)
        vx, vy = seg.x2 - seg.x1, seg.y2 - seg.y1
        wx, wy = robot.x - seg.x1, robot.y - seg.y1
        
        vlen2 = vx * vx + vy * vy
        if vlen2 == 0: continue
        
        t = max(0.0, min(1.0, (wx * vx + wy * vy) / vlen2))
        cx = seg.x1 + t * vx
        cy = seg.y1 + t * vy
        
        dx, dy = robot.x - cx, robot.y - cy
        d2 = dx * dx + dy * dy
        r = ROBOT_RADIUS + 2
        
        if d2 < r * r:
            d = math.sqrt(max(1e-9, d2))
            nx, ny = dx / d, dy / d
            push = (r - d)
            robot.x += nx * push
            robot.y += ny * push
            robot.theta = (robot.theta + 0.05) % math.tau
            pushed = True
            
    if pushed:
        robot.SetSpeed(0.0, 0.0)


def draw_lidar_rays(surf, robot: DiffDriveRobot, hits):
    """Zeichnet Rays. Hits ist jetzt ein NumPy Array (N, 3)."""
    # Wenn hits ein NumPy Array ist, iterieren wir darüber
    for val in hits:
        hx, hy, d = val[0], val[1], val[2]
        pygame.draw.line(surf, LIDAR_RAY_COLOR, (robot.x, robot.y), (hx, hy), 1)
        if d < LIDAR_MAX_RANGE * 0.999:
            pygame.draw.circle(surf, LIDAR_HIT_COLOR, (int(hx), int(hy)), 2)


def main():
    gate = Gate(
        gateWidthMin=120-40,    
        gateWidthMax=120+40,
        startPointDist=200,     
        maxWinkel=90,          
        vonRechts=True, 
        segBasedDetection=False,
        freeRangeDist=600
    )
        
    robot = DiffDriveRobot(x=180, y=100, theta=0) 
    robotController = RobotController(
            robot, 
            gate,
                gateReachedThreshold=30,            # Schwellwert für Erreichen des Tores
                startPointThreshold=10,             # Schwellwert für Erreichen Startpunktes
                baseSpeed = 70.0,                   # Basisfahrgeschwindigkeit [px/s]#
                kHeading = 2.2                      # Proportionalgain auf den Richtungsfehler
    )

    pygame.init()
    info = pygame.display.Info()
    screen_width = info.current_w
    screen_height = info.current_h
    
    # Falls Fullscreen gewünscht ist:
    # screen = pygame.display.set_mode((screen_width, screen_height), pygame.FULLSCREEN)
    # Für Fenstermodus (besser zum Debuggen):
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    
    pygame.display.set_caption("NumPy Optimized LiDAR Bot")
    clock = pygame.time.Clock()

    world = World()
    
    show_rays = True   
    manual = False     
    running = True
    debugMode = False
    first = True
    
    while running:
        dt = clock.tick(FPS) / 1000.0 

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_r:
                    show_rays = not show_rays
                elif event.key == pygame.K_m:
                    manual = not manual
                elif event.key == pygame.K_SPACE:
                    robot.Reset()
                    robotController.Reset()
                elif event.key == pygame.K_d:
                    debugMode = not debugMode

        screen.fill(BG_COLOR)
        world.draw(screen)

        # --- NumPy LiDAR ---
        lidar_hits, angles, radius = cast_lidar(world, robot.x, robot.y, robot.theta)

        if show_rays:
            draw_lidar_rays(screen, robot, lidar_hits)

        if manual:
            keys = pygame.key.get_pressed()
            fwd = 0.0
            turn = 0.0
            if keys[pygame.K_UP]: fwd += 1.0
            if keys[pygame.K_DOWN]: fwd -= 1.0
            if keys[pygame.K_LEFT]: turn -= 1.0
            if keys[pygame.K_RIGHT]: turn += 1.0
            
            vl = BASE_SPEED * fwd - turn * 40.0
            vr = BASE_SPEED * fwd + turn * 40.0
            robot.SetSpeed(vl, vr)
        else:
            # Wichtig: RobotLib2 muss mit NumPy Arrays umgehen können.
            # Falls nicht, hier .tolist() verwenden: radius.tolist()
            result = robotController.Run(angles, radius, dt)
            
            if result is not None and not robotController.Ready():
                torMitte, startPoint, pfosten1, pfosten2 = result
                robot.drawPoint(screen, pfosten1)
                robot.drawPoint(screen, pfosten2)
                robot.drawPoint(screen, torMitte)
                robot.drawPoint(screen, startPoint)

        if debugMode:
            if first:
                print(f"{robot.x=:.2f}  {robot.y=:.2f}")
                first = False
        else:
            first = True
            robot.step(dt)
            resolve_collisions(robot, world)

        robot.draw(screen)
        
        font = pygame.font.SysFont(None, 18)
        txt = (
            f"State: {robotController.GetState()}  "
            f"FPS: {clock.get_fps():.1f}  "
            f"Rays:{LIDAR_COUNT} (NumPy)"
        )
        # Visualisierung Tor-Mitte (fix)
        pygame.draw.circle(screen, GATE_COLOR, (WALL_X, (GATE_Y1 + GATE_Y2)//2), 4)

        # HUD zeichnen (mit Flip für Koordinatensystem, falls nötig)
        # Hier normal zeichnen:
        screen.blit(font.render(txt, True, (220, 220, 220)), (10, 10))
        
        pygame.display.flip()
        
        global numSteps
        numSteps += 1

    pygame.quit()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("Error:", e, file=sys.stderr)
        pygame.quit()
        raise