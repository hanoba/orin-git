# Dokumentation der TurtleBot‑ähnlichen PyGame‑Simulation

## Zweck und Überblick
Dieses Skript simuliert einen einfachen zweirädrigen Roboter (Differentialantrieb, ähnlich einem TurtleBot) in einer PyGame‑Umgebung. Der Roboter nutzt einen simulierten 360°‑LiDAR‑Sensor, um eine Öffnung (Tor) in einer Wand zu erkennen und autonom hindurchzufahren.

Die Simulation stellt eine rein kinematische Umgebung dar: Es gibt keine dynamischen Effekte wie Reibung oder Trägheit, sondern nur positionsbasierte Bewegungen.

---

## Hauptkomponenten

### 1. **Globale Parameter**
- Definition der Simulationsgeometrie (Fenstergröße, Wände, Torposition)
- LiDAR‑Einstellungen (Strahlzahl, Reichweite, Rauschen)
- Roboterparameter (Radradius, Spurweite, Maximalgeschwindigkeit)
- Reglerkonstanten für die Steuerung (z. B. `K_HEADING` für den P‑Regler)

---

### 2. **Weltmodell (`World`‑Klasse)**
Die Klasse `World` beschreibt die statische Umgebung:
- Vier äußere Wände (rechteckige Begrenzung)
- Eine vertikale Wand mit einer Öffnung (das Tor)

```python
class World:
    def __init__(self):
        self.segments = []
        self.add_rect_border(...)
        self.segments.append(Segment(WALL_X, y0, WALL_X, GATE_Y1))  # oberes Wandstück
        self.segments.append(Segment(WALL_X, GATE_Y2, WALL_X, y1))  # unteres Wandstück
```

Jede Wand wird durch ein `Segment`‑Objekt definiert (zwei Punkte).

---

### 3. **Segment‑Klasse**
Repräsentiert ein Liniensegment mit Start‑ und Endkoordinaten. Wird zum Zeichnen und zur Kollisionsprüfung verwendet.

```python
@dataclass
class Segment:
    x1, y1, x2, y2: float
```

---

### 4. **LiDAR‑Simulation (`cast_lidar`)**
Erzeugt 360 Strahlen (je 1° Abstand) um den Roboter herum. Jeder Strahl prüft Schnittpunkte mit Wänden:
```python
def cast_lidar(world, px, py, theta):
    for i in range(LIDAR_COUNT):
        ang = math.radians(i) + theta
        res = ray_segment_intersection(px, py, cos(ang), sin(ang), seg)
```
Ergebnis: Liste von Treffpunkten `(x, y, Distanz)`.

Die LiDAR‑Winkel sind **relativ zur Roboterorientierung**. Dadurch bleibt die Toröffnung in Roboterkoordinaten stabil.

---

### 5. **Ray‑Segment‑Schnitt**
Mathematische Berechnung, ob ein LiDAR‑Strahl ein Segment trifft. Wird in `cast_lidar()` genutzt.

```python
def ray_segment_intersection(px, py, dx, dy, seg):
    ... # liefert (ix, iy, t) oder None
```

---

### 6. **Differentialroboter‑Kinematik (`DiffDriveRobot`)**
Verwaltet Position, Orientierung und Bewegung des Roboters.

- `set_wheels(vl, vr)`: setzt Radgeschwindigkeiten mit Begrenzung.
- `step(dt)`: berechnet neue Pose anhand der Differentialgleichungen.
- `draw(surf)`: zeichnet Roboter und Orientierungslinie.

Kinematische Gleichungen:
\[ v = \frac{v_R + v_L}{2}, \quad \omega = \frac{v_R - v_L}{L} \]

---

### 7. **Torerkennung (`detect_gate`)**
Durchsucht das LiDAR‑Panorama nach der längsten freien Winkelzone:
```python
free = [1 if d >= GATE_FREE_RANGE_THRESH else 0 for (_, _, d) in lidar_hits]
```
Fügt die Liste zirkulär an sich selbst, um Öffnungen über 360° zu erkennen, und bestimmt den Mittelwinkel der größten freien Zone.

Ergebnis: `(mid_angle, length)` oder `None`.

---

### 8. **Controller‑Funktion**
Zentrale Logik für Autonomie. Zustandsmaschine mit drei Phasen:

| Zustand | Beschreibung |
|----------|---------------|
| `STATE_SEARCH` | Der Roboter pendelt vor der Wand und sucht eine Öffnung. |
| `STATE_ALIGN_AND_GO` | Er richtet sich auf das Tor aus und fährt darauf zu. |
| `STATE_DONE` | Nach Durchfahrt stoppt der Roboter. |

#### P‑Regler
```python
err = wrap_angle(gate[0] - robot.theta - math.pi / 2)
omega_cmd = K_HEADING * err
```
`err` ist der Richtungsfehler zwischen Roboterorientierung und Torwinkel. Der P‑Regler erzeugt eine Drehgeschwindigkeit `omega_cmd`. Diese wird in Radgeschwindigkeiten umgerechnet.

Der Zustand wechselt zu `STATE_DONE`, wenn der Roboter deutlich hinter der Wandlinie ist.

---

### 9. **Kollisionserkennung (`resolve_collisions`)**
Verhindert, dass der Roboter in Wände eindringt:
1. Berechnet den nächsten Punkt auf jedem Segment.
2. Wenn der Abstand kleiner als der Roboter‑Radius ist, wird der Roboter weggeschoben.

---

### 10. **Zeichenfunktionen**
- `draw_lidar_rays()` zeichnet Strahlen und Trefferpunkte.
- `World.draw()` zeichnet Umgebung.
- `DiffDriveRobot.draw()` zeichnet den Roboter.

---

### 11. **Main‑Schleife (`main()`)**
Steuert den gesamten Simulationsablauf:
- Initialisierung von PyGame und Weltobjekten
- Eingabeauswertung (ESC, M, R, SPACE)
- LiDAR‑Erfassung
- Moduswahl (manuell oder autonom)
- Aufruf von `controller()`
- Kinematikupdate (`robot.step(dt)`)
- Rendering aller Elemente

---

## Bedienung
| Taste | Funktion |
|-------|-----------|
| **ESC** | Programm beenden |
| **R** | LiDAR‑Strahlen ein/aus |
| **M** | Manueller Modus an/aus |
| **SPACE** | Roboterposition zurücksetzen |

---

## Ablauf
1. Roboter startet links von der Wand.
2. Sucht das Tor mittels LiDAR.
3. Erkennt freie Zone (Gate).
4. Richtet sich senkrecht darauf aus.
5. Fährt hindurch.
6. Stoppt hinter der Wand.

---

## Zusammenfassung
Dieses Skript zeigt:
- Umsetzung einer **nicht‑holonomen Kinematik** in 2D‑Simulation.
- Nutzung eines **simulierten 360°‑LiDARs** zur Umgebungswahrnehmung.
- Implementierung eines **P‑Reglers** zur Richtungssteuerung.
- Grundprinzipien der autonomen Navigation in PyGame.

---

**Ende der Dokumentation**
