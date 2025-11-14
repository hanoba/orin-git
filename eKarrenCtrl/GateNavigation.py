import math
import random

# --- KONSTANTEN ---
# Maximale Reichweite des simulierten LiDARs (in Metern)
MAX_RANGE = 5.0
# Schwelle, um festzustellen, ob ein Bereich frei (Tor) oder blockiert (Mauer) ist.
# Alles über diesem Wert wird als Teil eines potenziellen Tores betrachtet.
GATE_THRESHOLD = 2.0
# Die Toleranz (in Grad) für die Ausrichtung, bevor geradeaus gefahren wird.
# Liegt der Mittelpunkt des Tores innerhalb dieser Toleranz, gilt der Roboter als ausgerichtet (rechter Winkel).
ALIGNMENT_TOLERANCE = 2.0 

class LidarNavigator:
    """
    Simuliert einen Roboter mit einem 360° LiDAR, der ein Tor in einer Wand sucht
    und die notwendigen Motorbefehle ableitet, um im rechten Winkel hindurchzufahren.
    """

    def __init__(self, num_points=360):
        self.num_points = num_points
        self.angle_step = 360 / num_points
        self.current_lidar_data = {} # {Grad: Entfernung}

    def simulate_scan(self, gate_start_angle=340, gate_end_angle=20, wall_distance=0.5):
        """
        Simuliert einen 360°-LiDAR-Scan.
        Die Wand ist in der Nähe (wall_distance), das Tor ist weit weg (MAX_RANGE).
        
        Der Scan geht von 0 bis 359 Grad. 0 Grad ist direkt vor dem Roboter.
        """
        data = {}
        for i in range(self.num_points):
            angle = i * self.angle_step

            is_in_gate_range = False
            
            # Prüfen, ob der Winkel im Torbereich liegt.
            # Berücksichtigt den Überlauf von 359 zu 0 Grad (z.B. Tor bei 340-20 Grad)
            if gate_start_angle < gate_end_angle: # Normaler Fall (z.B. 100 bis 200)
                if gate_start_angle <= angle <= gate_end_angle:
                    is_in_gate_range = True
            else: # Überlauf (z.B. 340 bis 20)
                if angle >= gate_start_angle or angle <= gate_end_angle:
                    is_in_gate_range = True

            if is_in_gate_range:
                # Tor: Große Entfernung (plus etwas Zufall für Realismus)
                data[angle] = MAX_RANGE + random.uniform(0.1, 0.5)
            else:
                # Wand: Geringe Entfernung (plus etwas Zufall für Realismus)
                data[angle] = wall_distance + random.uniform(-0.1, 0.1)

        self.current_lidar_data = data
        return data

    def find_best_gate(self):
        """
        Analysiert die LiDAR-Daten, um das größte freie Segment (Tor) zu finden.
        Gibt den Start-, End- und Mittelpunktwinkel des größten Tores zurück.
        """
        points = list(self.current_lidar_data.items())
        
        # 1. Daten "entfalten", um den Übergang von 359° zu 0° zu behandeln.
        # Wir fügen die ersten 60 Punkte am Ende hinzu (ca. 60 Grad), 
        # um sicherzustellen, dass ein Tor, das bei 350° beginnt und bei 10° endet, 
        # als ein einziges, zusammenhängendes Tor erkannt wird.
        points_extended = points + points[:60]

        gaps = []
        is_in_gap = False
        current_gap_start_index = -1

        for i, (angle, distance) in enumerate(points_extended):
            is_clear = distance >= GATE_THRESHOLD

            if is_clear and not is_in_gap:
                # Start eines neuen Tores/Gap
                is_in_gap = True
                current_gap_start_index = i
            
            elif not is_clear and is_in_gap:
                # Ende des aktuellen Tores/Gap
                is_in_gap = False
                end_index = i - 1
                start_angle = points_extended[current_gap_start_index][0]
                end_angle = points_extended[end_index][0]
                
                # Nur speichern, wenn das Ende nicht im "erweiterten" Teil liegt
                if end_index < self.num_points:
                    gaps.append({
                        "start_index": current_gap_start_index,
                        "end_index": end_index,
                        "length": end_index - current_gap_start_index,
                        "start_angle": start_angle,
                        "end_angle": end_angle
                    })
        
        if not gaps:
            return None, None, None # Kein Tor gefunden

        # 2. Finde das größte Tor
        best_gap = max(gaps, key=lambda g: g["length"])
        
        start_angle = best_gap["start_angle"]
        end_angle = best_gap["end_angle"]
        
        # Berechnung des Mittelpunkts.
        # Wichtig: Muss den 360°-Übergang korrekt behandeln!
        center_angle = (start_angle + end_angle) / 2
        
        # Wenn das Tor über 360/0 Grad liegt (z.B. 350° bis 10°), 
        # wird die einfache Mittelung falsch sein (360/2 = 180°).
        # Hier korrigieren wir den Winkel so, dass er im Bereich [-180, 180] liegt.
        if start_angle > end_angle:
            center_angle = (start_angle + end_angle + 360) / 2
            center_angle = center_angle % 360 # Wrap around
        
        # Der Mittelpunkt sollte jetzt im Bereich [0, 360] sein.
        # Wir verschieben ihn in den Bereich [-180, 180], wobei 0 vorne ist.
        if center_angle > 180:
            center_angle -= 360

        return start_angle, end_angle, center_angle

    def get_motor_commands(self, center_angle):
        """
        Leitet aus dem Zielwinkel die Fahrbefehle ab. Die Strategie ist:
        1. Drehen, bis der Roboter im rechten Winkel zum Tor steht (Ausrichtung).
        2. Geradeaus fahren.

        Gibt Geschwindigkeiten für den linken und rechten Motor und die Aktion zurück.
        """
        # Geschwindigkeit für das Fahren durch das Tor
        drive_speed = 0.5
        # Geschwindigkeit für das Drehen (reine Drehung, am Stand)
        turn_speed = 0.3
        
        # Die Toleranz wird von der Konstante oben verwendet
        alignment_tolerance = ALIGNMENT_TOLERANCE 
        
        if center_angle is None:
            # Fall: Kein Tor gefunden -> Stoppen
            return 0.0, 0.0, "SUCHE: Tor nicht gefunden (Stopp)."

        # --- PHASE 1: AUSRICHTUNG (DREHEN) ---
        # Drehen, bis der Mittelpunkt des Tores (center_angle) nahe 0° ist (rechter Winkel zur Wand)
        if abs(center_angle) > alignment_tolerance:
            
            # Tor befindet sich links (negativer Winkel) -> Drehung nach links
            if center_angle < 0:
                # Linker Motor rückwärts, Rechter Motor vorwärts (Drehung am Stand)
                return -turn_speed, turn_speed, f"AUSRICHTUNG: Drehen nach links um {center_angle:.1f}°."
            
            # Tor befindet sich rechts (positiver Winkel) -> Drehung nach rechts
            else:
                # Linker Motor vorwärts, Rechter Motor rückwärts (Drehung am Stand)
                return turn_speed, -turn_speed, f"AUSRICHTUNG: Drehen nach rechts um {center_angle:.1f}°."

        # --- PHASE 2: DURCHFAHREN (GERADEAUS FAHREN) ---
        else:
            # Roboter ist ausgerichtet (innerhalb der Toleranz) und fährt geradeaus
            # Die geforderte Mindeststrecke von 3 Metern wird in der Simulation 
            # durch diesen Befehl repräsentiert.
            return drive_speed, drive_speed, f"DURCHFAHRT: Im rechten Winkel 3m fahren (Ausrichtung: {center_angle:.1f}°)."

# --- HAUPTPROGRAMM ---
def main():
    """Simuliert den Navigationsprozess."""
    print("--- LiDAR Navigationssystem (YOLOv11-ähnlicher Roboter) ---")
    print("Ziel: Ausrichtung im rechten Winkel zum Tor, dann 3 Meter geradeaus fahren.")
    navigator = LidarNavigator()

    # --- SIMULATIONSSZENARIO 1: TOR IST LEICHT RECHTS ---
    print("\n[Simulations-Setup 1]: Tor ist leicht rechts (350° bis 30°)...")
    navigator.simulate_scan(gate_start_angle=350, gate_end_angle=30, wall_distance=0.7)

    start_angle, end_angle, center_angle = navigator.find_best_gate()
    left_speed, right_speed, action = navigator.get_motor_commands(center_angle)

    print(f"  > Erkanntes Tor: Start {start_angle:.1f}° | Ende {end_angle:.1f}°")
    print(f"  > Zielwinkel (Mitte): {center_angle:.1f}°")
    
    # Simulation des Ablaufs: Zuerst drehen, dann fahren.
    if "AUSRICHTUNG" in action:
        print(f"  > **PHASE 1 (Drehung):** Befehl: {action} (L:{left_speed:.2f} / R:{right_speed:.2f})")
        
        # Simuliere den nächsten Befehl (wenn Ausrichtung abgeschlossen ist)
        left_speed_drive, right_speed_drive, action_drive = navigator.get_motor_commands(0.0)
        print(f"  > **PHASE 2 (Fahrt):** Befehl: {action_drive} (L:{left_speed_drive:.2f} / R:{right_speed_drive:.2f})")
    else:
        # Fall 1: Tor bereits perfekt zentriert
        print(f"  > **PHASE 1 & 2 (Fahrt):** Befehl: {action} (L:{left_speed:.2f} / R:{right_speed:.2f})")


    # --- SIMULATIONSSZENARIO 2: TOR IST ZENTRIERT ---
    print("\n[Simulations-Setup 2]: Tor ist zentriert (340° bis 20°)...")
    navigator.simulate_scan(gate_start_angle=340, gate_end_angle=20, wall_distance=0.7)

    start_angle, end_angle, center_angle = navigator.find_best_gate()
    left_speed, right_speed, action = navigator.get_motor_commands(center_angle)

    print(f"  > Erkanntes Tor: Start {start_angle:.1f}° | Ende {end_angle:.1f}°")
    print(f"  > Zielwinkel (Mitte): {center_angle:.1f}°")

    if "AUSRICHTUNG" in action:
        print(f"  > **PHASE 1 (Drehung):** Befehl: {action} (L:{left_speed:.2f} / R:{right_speed:.2f})")
        left_speed_drive, right_speed_drive, action_drive = navigator.get_motor_commands(0.0)
        print(f"  > **PHASE 2 (Fahrt):** Befehl: {action_drive} (L:{left_speed_drive:.2f} / R:{right_speed_drive:.2f})")
    else:
        print(f"  > **PHASE 1 & 2 (Fahrt):** Befehl: {action} (L:{left_speed:.2f} / R:{right_speed:.2f})")

    
    # --- SIMULATIONSSZENARIO 3: TOR IST DEUTLICH LINKS ---
    print("\n[Simulations-Setup 3]: Tor ist deutlich links (100° bis 150°)...")
    navigator.simulate_scan(gate_start_angle=100, gate_end_angle=150, wall_distance=0.7)

    start_angle, end_angle, center_angle = navigator.find_best_gate()
    left_speed, right_speed, action = navigator.get_motor_commands(center_angle)

    print(f"  > Erkanntes Tor: Start {start_angle:.1f}° | Ende {end_angle:.1f}°")
    print(f"  > Zielwinkel (Mitte): {center_angle:.1f}°")

    if "AUSRICHTUNG" in action:
        print(f"  > **PHASE 1 (Drehung):** Befehl: {action} (L:{left_speed:.2f} / R:{right_speed:.2f})")
        left_speed_drive, right_speed_drive, action_drive = navigator.get_motor_commands(0.0)
        print(f"  > **PHASE 2 (Fahrt):** Befehl: {action_drive} (L:{left_speed_drive:.2f} / R:{right_speed_drive:.2f})")
    else:
        print(f"  > **PHASE 1 & 2 (Fahrt):** Befehl: {action} (L:{left_speed:.2f} / R:{right_speed:.2f})")


if __name__ == "__main__":
    main()
