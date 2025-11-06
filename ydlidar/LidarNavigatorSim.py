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
import random
import sys
from dataclasses import dataclass
import pygame
from RobotLib import Gate, RobotController

numSteps = 0

# =========================
# Globale Parameter / Konfiguration
# =========================
WIN_W, WIN_H = 900, 600              # Fenstergröße in Pixeln
FPS = 10                             # Simulationsrate (Frames pro Sekunde)
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
BORDER_MARGIN = 40                   # Abstand der Begrenzungswände vom Fenster
WALL_X = 500                         # X‑Position der vertikalen Mauer
GATE_Y1 = 240                        # oberes Ende der Toröffnung
GATE_Y2 = 360                        # unteres Ende der Toröffnung

# LiDAR‑Parameter
LIDAR_COUNT = 180      #HB 360       # Anzahl der Strahlen (1° Raster)
LIDAR_MAX_RANGE = 320.0*3            # maximale Messdistanz in Pixeln
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


@dataclass
class Segment:
    """Liniensegment (Wand/Begrenzung) in Weltkoordinaten."""
    x1: float
    y1: float
    x2: float
    y2: float

    def draw(self, surf, color):
        pygame.draw.line(surf, color, (self.x1, self.y1), (self.x2, self.y2), 2)


class World:
    """Enthält alle kollidierenden Liniensegmente und zeichnet die Szene."""
    def __init__(self):
        self.segments = []
        # Außenrahmen erzeugen
        x0, y0 = BORDER_MARGIN, BORDER_MARGIN
        x1, y1 = WIN_W - BORDER_MARGIN, WIN_H - BORDER_MARGIN
        x1 = WALL_X     #HB
        self.add_rect_border(x0, y0, x1, y1)
        # Vertikale Mauer, Tor wird als Lücke modelliert: zwei Segmente oben/unten
        self.segments.append(Segment(WALL_X, y0, WALL_X, GATE_Y1))
        self.segments.append(Segment(WALL_X, GATE_Y2, WALL_X, y1))

    def add_rect_border(self, x0, y0, x1, y1):
        self.segments += [
            Segment(x0, y0, x1, y0),  # oben
            #Segment(x1, y0, x1, y1),  # rechts
            Segment(x1, y1, x0, y1),  # unten
            Segment(x0, y1, x0, y0),  # links
        ]

    def draw(self, surf):
        for s in self.segments:
            s.draw(surf, WALL_COLOR)
        # Tor optisch hervorheben (nur Visualisierung, keine Kollision)
        pygame.draw.line(surf, GATE_COLOR, (WALL_X, GATE_Y1), (WALL_X, GATE_Y2), 3)


# -------------------------
# Geometrie: Ray/Segment‑Schnitt
# -------------------------
# Ray: P + t*d, t>=0; Segment: S + u*r, u∈[0,1]
# Liefert (ix,iy,t) mit t als Entfernungsskalar entlang des Strahls. None falls kein Schnitt.

def ray_segment_intersection(px, py, dx, dy, seg: Segment):
    sx, sy = seg.x1, seg.y1
    rx, ry = seg.x2 - seg.x1, seg.y2 - seg.y1
    denom = dx * ry - dy * rx
    if abs(denom) < 1e-9:
        return None  # parallel oder kollinear
    t = ((sx - px) * ry - (sy - py) * rx) / denom
    if t < 0:
        return None  # Schnitt hinter dem Ray‑Ursprung
    u = ((sx - px) * dy - (sy - py) * dx) / denom
    if u < 0 or u > 1:
        return None  # außerhalb des Segments
    ix = px + t * dx
    iy = py + t * dy
    return (ix, iy, t)


# -------------------------
# LiDAR‑Simulation
# -------------------------
# Wichtig: Der Strahlwinkel wird relativ zur Roboterorientierung theta aufgebaut.
# Dadurch „wandern“ Wände im LiDAR‑Panorama, wenn sich der Roboter dreht, und das Gate
# bleibt als zusammenhängende freie Winkelmenge erkennbar.

# Die Rückgabewerte von cast_lidar sind in Weltkoordinaten (globale X-/Y-Positionen), da 
# die Strahlenrichtung relativ zur Roboterorientierung berechnet, aber anschließend zur
# globalen Roboterposition addiert wird.
#
def cast_lidar(world: World, px, py, theta=0.0):
    hits = []
    angles = []
    radius = []
    i0 = LIDAR_COUNT // 2
    for i in range(LIDAR_COUNT):
        # Winkel in Rad: i° + theta, 0° zeigt in Roboter‑Vorwärtsrichtung (x‑Achse)
        ang = math.radians(i - i0) + theta
        dx, dy = math.cos(ang), math.sin(ang)
        best_t = None
        # Finde nächstgelegenen Schnitt mit irgendeinem Segment
        for seg in world.segments:
            res = ray_segment_intersection(px, py, dx, dy, seg)
            if res is None:
                continue
            _, _, t = res
            if best_t is None or t < best_t:
                best_t = t
        # Distanz begrenzen; wenn kein Treffer, setze MAX_RANGE
        dist = LIDAR_MAX_RANGE if best_t is None else min(best_t, LIDAR_MAX_RANGE)
        # Einfaches Messrauschen
        noisy_dist = max(0.0, min(LIDAR_MAX_RANGE, dist + random.gauss(0, LIDAR_NOISE_STD)))
        hits.append((px + dx * noisy_dist, py + dy * noisy_dist, noisy_dist))
        #angles.append(i/LIDAR_COUNT*math.pi - 0*math.pi/2)
        angles.append(ang - theta)
        radius.append(noisy_dist)
    return hits, angles, radius  # Liste von (HitX, HitY, Distanz)


class DiffDriveRobot:
    """Sehr einfache Kinematik eines Differenzialroboters."""
    # theta beschreibt die Roboterorientierung im Weltkoordinatensystem 
    # (0 rad nach rechts, positive Rotation gegen den Uhrzeigersinn, gemessen in Radiant).
    def __init__(self, x, y, theta):
        self.x = self.x0 = x
        self.y = self.y0 = y
        self.theta = self.theta0 = theta    # Orientierung in Rad (0 = nach rechts)
        self.v_l = 0.0                      # linkes Rad [px/s]
        self.v_r = 0.0                      # rechtes Rad [px/s]
        self.state = STATE_SEARCH           # Startzustand
        self.target_angle = -math.pi   #HB None    # Zielrichtung (in Roboter‑Koordinaten ermittelt)

    def Reset(self):
        """ Robotor auf Startposition/ausrichting setzen """
        self.x = self.x0 
        self.y = self.y0 
        self.theta = self.theta0 
        self.state = STATE_SEARCH
        self.set_wheels(0, 0)

    def set_wheels(self, vl, vr):
        """Setzt Radgeschwindigkeiten mit Sättigung."""
        m = MAX_WHEEL_SPEED
        self.v_l = max(-m, min(m, vl))
        self.v_r = max(-m, min(m, vr))

    def step(self, dt):
        """Update der Pose per nicht‑holonomer Kinematik."""
        v = (self.v_r + self.v_l) * 0.5
        omega = (self.v_r - self.v_l) / WHEEL_BASE
        # Winkel normieren auf [0, 2π)
        self.theta = (self.theta + omega * dt + math.tau) % math.tau
        #if numSteps < 20: print(f"{omega=}  {self.theta=}")     #HB
        # Positionsupdate in Weltkoordinaten
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

    def draw(self, surf):
        """Zeichnet Roboterkörper als Kreis plus Heading‑Strich."""
        pygame.draw.circle(surf, ROBOT_COLOR, (int(self.x), int(self.y)), ROBOT_RADIUS, 2)
        hx = self.x + math.cos(self.theta) * ROBOT_RADIUS
        hy = self.y + math.sin(self.theta) * ROBOT_RADIUS
        pygame.draw.line(surf, ROBOT_COLOR, (self.x, self.y), (hx, hy), 2)

    def drawPoint(self, surf, point):
        """ Zeichnet einen berechneten Punkt """
        p = point*cmath.exp(1j*self.theta)
        x = int(self.x+p.real)
        y = int(self.y+p.imag)
        pygame.draw.circle(surf, POINT_COLOR, (x, y), POINT_RADIUS, 2)

def G(x):
    return x*360/math.tau


def resolve_collisions(robot: DiffDriveRobot, world: World):
    """Einfache Kollisionsauflösung: trennt den Roboter von segmentnahen Punkten.

    Vorgehen
      1) Projektion des Roboterzentrums auf jedes Segment
      2) Falls Abstand < ROBOT_RADIUS, schiebe entlang der Normalen heraus
      3) leichte Winkeländerung als numerische Hilfe
    """
    pushed = False
    for seg in world.segments:
        vx, vy = seg.x2 - seg.x1, seg.y2 - seg.y1
        wx, wy = robot.x - seg.x1, robot.y - seg.y1
        vlen2 = vx * vx + vy * vy
        if vlen2 == 0:
            continue
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
        robot.set_wheels(0.0, 0.0)


def draw_lidar_rays(surf, robot: DiffDriveRobot, hits):
    """Zeichnet alle LiDAR-Strahlen und markiert Trefferpunkte."""
    for hx, hy, d in hits:
        pygame.draw.line(surf, LIDAR_RAY_COLOR, (robot.x, robot.y), (hx, hy), 1)
        if d < LIDAR_MAX_RANGE * 0.999:
            pygame.draw.circle(surf, LIDAR_HIT_COLOR, (int(hx), int(hy)), 2)


def main():
    # Setup gate to be detected
    gate = Gate(
        gateWidthMin=120-40,    # 0.275, 
        gateWidthMax=120+40,
        startPointDist=200,     # Abstand zwischen Startpunkt und Tor 0.4, 
        maxWinkel = 90,         #80,
        vonRechts=True, 
        segBasedDetection=False,
        freeRangeDist=600)
        
    robot = DiffDriveRobot(x=180, y=100, theta=-3.14/4*0)  #HB
    robotController = RobotController(
                robot.set_wheels, 
                gate,
                gateReachedThreshold=30,            # Schwellwert für Erreichen des Tores
                startPointThreshold=10,             # Schwellwert für Erreichen Startpunktes
                wheelBase = 2 * 16,                 # Radabstand (vereinfacht)
                baseSpeed = 70.0,                   # Basisfahrgeschwindigkeit [px/s]#
                kHeading = 2.2                      # Proportionalgain auf den Richtungsfehler
    )

    # PyGame-Setup
    pygame.init()
    
    # Get the display information object
    info = pygame.display.Info()

    # Access the width and height attributes
    screen_width = info.current_w
    screen_height = info.current_h
    screen_resolution = (screen_width, screen_height)

    print(f"Detected Screen Width: {screen_width} pixels")
    print(f"Detected Screen Height: {screen_height} pixels")
    
    #screen = pygame.display.set_mode((WIN_W, WIN_H))
    screen = pygame.display.set_mode(screen_resolution, pygame.FULLSCREEN)
    
    pygame.display.set_caption("TurtleBot-ähnlicher LiDAR-Gate-Navigator (PyGame)")
    clock = pygame.time.Clock()

    # Szenenaufbau
    world = World()

    # Startpose: links der Mauer, unterhalb des Tores

    show_rays = True   # LiDAR‑Darstellung an/aus
    manual = False     # manueller Modus per Pfeiltasten

    running = True
    debugMode = False
    first = True
    while running:
        result = None
        dt = clock.tick(FPS) / 1000.0 / 1 #HB # Zeitschritt in Sekunden
        # Eingaben verarbeiten
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
                    # vollständiger Reset der Roboterpose und des Zustands
                    robot.Reset()
                    robotController.Reset()
                elif event.key == pygame.K_d:
                    debugMode = not debugMode

        # Rendering der Welt
        screen.fill(BG_COLOR)
        world.draw(screen)

        # LiDAR erzeugen: Wichtig, theta mitgeben, damit Strahlen im Roboterframe sind
        lidar_hits, angles, radius = cast_lidar(world, robot.x, robot.y, robot.theta)

        # LiDAR zeichnen
        if show_rays:
            draw_lidar_rays(screen, robot, lidar_hits)

        # Manuelle Steuerung: sehr simpel, Pfeile → Vorwärts/ Rückwärts/ Drehen
        if manual:
            keys = pygame.key.get_pressed()
            fwd = 0.0
            turn = 0.0
            if keys[pygame.K_UP]:
                fwd += 1.0
            if keys[pygame.K_DOWN]:
                fwd -= 1.0
            if keys[pygame.K_LEFT]:
                turn -= 1.0
            if keys[pygame.K_RIGHT]:
                turn += 1.0
            vl = BASE_SPEED * fwd - turn * 40.0
            vr = BASE_SPEED * fwd + turn * 40.0
            robot.set_wheels(vl, vr)
        else:
            # Autopilot
            result = robotController.Run(angles, radius, dt)
            
            # Visualisierung von Tor und Startpunkt
            if result is not None and not robotController.Done():
                torMitte, startPoint, pfosten1, pfosten2 = result
                robot.drawPoint(screen, pfosten1)
                robot.drawPoint(screen, pfosten2)
                robot.drawPoint(screen, torMitte)
                robot.drawPoint(screen, startPoint)

        # Kinematikschritt + einfache Kollisionsauflösung
        if debugMode:
            if first:
                print(f"{startPoint=}   {G(cmath.phase(startPoint))}°")
                print(f"{torMitte=}   {G(cmath.phase(torMitte))}°")
                print(f"{robot.x=}    {robot.y=}")
                first = False
        else:
            first = True
            robot.step(dt)
            resolve_collisions(robot, world)

        # Roboter zeichnen + HUD
        robot.draw(screen)
        font = pygame.font.SysFont(None, 18)
        txt = (
            f"State: {robotController.GetState()}  "
            f"SPACE=Reset  M=Manual:{manual}  R=Rays:{show_rays}"
            f" {G(robot.target_angle):.0f}°"
        )
        pygame.draw.circle(screen, GATE_COLOR, (WALL_X, (GATE_Y1 + GATE_Y2)//2), 4)
        #if result != None:
        #    robot.drawPoint(screen, torMitte)
        #    robot.drawPoint(screen, startPoint)
        screen.blit(pygame.transform.flip(screen, False, True), (0, 0))
        screen.blit(font.render(txt, True, (220, 220, 220)), (10, 10))
        pygame.display.flip()
        
        global numSteps
        numSteps += 1

    pygame.quit()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        # Fehlerausgabe auf STDERR, danach sauber beenden und erneut auslösen
        print("Error:", e, file=sys.stderr)
        pygame.quit()
        raise
