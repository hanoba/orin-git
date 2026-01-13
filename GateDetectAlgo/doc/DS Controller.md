# Beschreibung der Funktion `controller(robot: DiffDriveRobot, lidar_hits, dt)`

## Überblick
Die Funktion `controller` ist das Steuerungszentrum des Roboters. Sie wertet die LiDAR-Daten aus, erkennt eine freie Öffnung (Tor) in der Mauer und berechnet daraus die Raddrehgeschwindigkeiten. Der Roboter wird in einer einfachen Zustandsmaschine gesteuert.

---

## Zustände
| Zustand | Beschreibung |
|----------|---------------|
| **STATE_SEARCH** | Der Roboter sucht aktiv nach einer freien Öffnung, indem er pendelnd fährt. |
| **STATE_ALIGN_AND_GO** | Der Roboter richtet sich auf das erkannte Tor aus und fährt darauf zu. |
| **STATE_DONE** | Der Roboter hat das Tor passiert und bleibt stehen. |

---

## Funktionsweise im Detail

### 1. Torerkennung
```python
gate = detect_gate(lidar_hits)
```
Die Funktion `detect_gate()` sucht in den 360°-LiDAR-Daten nach der größten freien Öffnung. Diese liefert den mittleren Öffnungswinkel `gate[0]` zurück (im Roboterkoordinatensystem).

---

### 2. SEARCH-Zustand
```python
if robot.state == STATE_SEARCH:
    if gate is not None:
        robot.target_angle = gate[0]
        robot.state = STATE_ALIGN_AND_GO
    bias = 20.0 * math.sin(pygame.time.get_ticks() * 0.001)
    robot.set_wheels(BASE_SPEED * 0.6 - bias, BASE_SPEED * 0.6 + bias)
```
- Der Roboter pendelt leicht, um die Umgebung zu scannen.
- Sobald ein Tor erkannt wird (`gate != None`), speichert er den Zielwinkel und wechselt in den Zustand **ALIGN_AND_GO**.

---

### 3. ALIGN_AND_GO-Zustand
```python
err = wrap_angle(gate[0] - robot.theta - math.pi / 2)
omega_cmd = K_HEADING * err
v_cmd = BASE_SPEED
vl = v_cmd - omega_cmd * (WHEEL_BASE / 2.0)
vr = v_cmd + omega_cmd * (WHEEL_BASE / 2.0)
robot.set_wheels(vl, vr)
```
- Berechnet den **Richtungsfehler** zwischen aktueller Orientierung `theta` und Zielwinkel `gate[0]`.
- Wendet einen **P-Regler** an, um die Drehgeschwindigkeit `omega_cmd` zu bestimmen.
- Kombiniert konstante Vorwärtsfahrt (`v_cmd`) mit Drehung, um das Tor zu zentrieren.
- Setzt die resultierenden Radgeschwindigkeiten.

**Stoppbedingung:**
```python
if robot.x > WALL_X + 80:
    robot.state = STATE_DONE
    robot.set_wheels(0, 0)
```
Wenn der Roboter die Mauerlinie überschritten hat, stoppt er.

---

### 4. DONE-Zustand
```python
if robot.state == STATE_DONE:
    robot.set_wheels(0, 0)
```
Der Roboter bleibt stehen und beendet die Bewegung.

---

## Zusammenfassung
| Schritt | Aufgabe |
|----------|----------|
| 1 | Gate-Erkennung mit LiDAR |
| 2 | Zustandsermittlung (SEARCH, ALIGN_AND_GO, DONE) |
| 3 | P-Regelung des Richtungsfehlers |
| 4 | Umrechnung auf linkes/rechtes Rad |
| 5 | Stoppen hinter dem Tor |

**Ergebnis:** Der Roboter findet das Tor, richtet sich senkrecht darauf aus und fährt selbstständig hindurch.