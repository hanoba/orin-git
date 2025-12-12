#!/usr/bin/env python3
import numpy as np
import math


class MyWallFollower:
    def __init__(self, target_dist=0.5, max_speed=0.5):
        self.target_dist = target_dist
        self.max_speed = max_speed

    def get_sector(self, dist, start, end):
        """Extrahiert Sektor über 0°-Überlauf korrekt."""
        if start <= end:
            return dist[start:end]
        else:
            return np.concatenate((dist[start:360], dist[0:end]))

    def step(self, dist):
        """
        dist: numpy array mit 360 floats (Entfernungen)
        Rückgabe: (v, omega)
          v: Vorwärtsgeschwindigkeit
          omega: Drehgeschwindigkeit (+ links, - rechts)
        """

        # --- 1) Wichtige Sektoren ---
        front    = self.get_sector(dist, 350, 10)   # -10° bis +10°
        right    = self.get_sector(dist, 300, 340)
        front_r  = self.get_sector(dist, 330, 350)

        d_front   = np.nanmin(front)
        d_right   = np.nanmin(right)
        d_front_r = np.nanmin(front_r)

        # --- 2) Parameter ---
        wall = self.target_dist
        v = self.max_speed

        # --- 3) Reaktionslogik ---

        w = 0.2

        #e = d_right - wall
        #Kp = 1.0
        #omegaMax = 0.1
        #omega = -e*Kp
        #omega = max(-omegaMax, omega)
        #omega = min(omegaMax, omega)
        #
        #Kv = 1.0
        #v = abs(e)*Kv
        #v = min(self.max_speed, v)
        #return v, omega
        
        # 3a) Hindernis direkt vorne → nach links drehen
        if d_front < 0.4:
            #print(f"Hindernis direkt vorne {d_front=}")
            return 0.0, w

        # 3b) Wand rechts zu nah → nach links korrigieren
        if d_right < wall * 0.95:
            #print(f"rechts zu nah {d_right=}")
            return v * 0.6/6, w*2

        # 3c) Wand rechts zu weit weg → nach rechts korrigieren
        if d_right > wall * 1.06:
            #print(f"rechts zu weit {d_right=}")
            return v * 0.6, -w*0.6

        if d_right > wall * 1.05:
            return v * 0.6/6, -w*2

        # 3d) Rechts-Front Ecke → leicht links
        if d_front_r < 0.5:
            #print(f"rechts-front Ecke {d_front_r=}")
            return v * 0.4, w*0.5

        # 3e) Normalfahrt → geradeaus
        return v, 0.0


SEARCH = 0
FOLLOW = 1
AVOID  = 2
NONE   = 3


class WallFollowerFinal:
    def __init__(self, target_dist=0.5, max_speed=0.35):
        # Zielabstand zur rechten Wand [m]
        self.target_dist = target_dist
        self.cos50 = math.cos(50/180*math.pi)   # winkel zwischen Wandabstand und right_front
        self.cos10 = math.cos(10/180*math.pi)   # winkel zwischen Wandabstand und right 

        # Basis-Vorwärtsgeschwindigkeit
        self.max_speed = max_speed

        # PD-Gewichte
        self.K_lat  = 8.0  # 2.0  # lateral error (Abstand)
        self.K_head = 8.0  # 1.0 # heading error (Wandorientierung)

        # Schwellenwerte
        self.front_thresh = 0.5   # Kollisionsschutz vorne
        self.far_thresh   = 2*self.target_dist  #  1.2    # ab hier gilt "Wand verloren"

        print("Final WallFollower started (StateMachine + PD).")

        self.state = NONE
        self.SetState(SEARCH)

    def GetStateText(self, newState):
        if newState==SEARCH: return "SEARCH"
        elif newState==FOLLOW: return "FOLLOW"
        elif newState==AVOID: return "AVOID"
        return "Unknown"
    
    def SetState(self, newState):
        if newState != self.state:
            print(f"State: {self.GetStateText(newState)}")
            self.state = newState

    # Hilfsfunktion: Sektor mit Wraparound in Grad
    def get_sector_deg(self, ranges_np, start_deg, end_deg):
        """Gibt einen Bereich in [start_deg, end_deg) zurück, Grad, 0..359, mit Wraparound."""
        start_deg %= 360
        end_deg   %= 360
        if start_deg <= end_deg:
            return ranges_np[start_deg:end_deg]
        else:
            return np.concatenate((ranges_np[start_deg:], ranges_np[:end_deg]))

    def step(self, ranges):
        r = np.array(ranges, dtype=float)

        # Inf / 0 als NaN behandeln
        r[~np.isfinite(r)] = np.nan
        r[r <= 0.0] = np.nan

        # Falls der Scan nicht 360 Werte hat, auf 360 „normalisieren“
        # (vereinfachend: bei den meisten 360°-Scannern ist das der Fall,
        #  sonst müsste man über angle_min/angle_increment gehen).
        if r.size != 360:
            print(f"ERROR {r.size=}")
            # grobe, aber robuste Resampling-Lösung
            idx = np.linspace(0, r.size - 1, 360).astype(int)
            r = r[idx]

        # --- Sektoren (Standard: 0° vorne, 90° links, 270° rechts) ---
        front_sector     = self.get_sector_deg(r, 350, 10)   # -10..+10°
        right_sector     = self.get_sector_deg(r, 270, 290)  # 280° +/- 10° rechts seitlich
        right_front_sec  = self.get_sector_deg(r, 310, 330)  # 320° +/- 10° rechts vorne

        d_front     = np.nanmin(front_sector) if np.any(~np.isnan(front_sector)) else np.inf
        d_right     = np.nanmean(right_sector) if np.any(~np.isnan(right_sector)) else np.inf
        #d_right_fwd = np.nanmin(right_front_sec) if np.any(~np.isnan(right_front_sec)) else np.inf
        d_right_fwd = np.nanmean(right_front_sec) if np.any(~np.isnan(right_front_sec)) else np.inf

        d_dist = d_right*self.cos10
        print(f"{self.GetStateText(self.state)} {d_front=}  {d_dist=}  {d_right_fwd=}")

        # -------- Zustandswahl --------
        # 1) Hindernis vorne: immer AVOID
        if d_front < self.front_thresh:
            self.SetState(AVOID)
        else:
            # Wand verloren? → SEARCH
            th = self.far_thresh
            if self.state == FOLLOW: th=th*1.2  # Hysterese
            if d_dist > th or np.isinf(d_right):
                self.SetState(SEARCH)
            else:
                self.SetState(FOLLOW)

        # -------- Zustandsverhalten --------
        if self.state == AVOID:
            # Hindernis vorne → stehen bleiben, links drehen
            linear  = 0.0
            angular = 1.0

        elif self.state == SEARCH:
            # Wand suchen:
            # sanfte Rechtskurve, damit man nicht ewig parallel fährt,
            # aber auch keine engen Kreise.
            linear  = 0.25
            angular = -0.2   # -0.25
            if d_right_fwd < d_right: angular = 0.1  #HB

        elif self.state == FOLLOW:
            # Nur wenn Wandmessung vernünftig
            if np.isfinite(d_right):
                # Lateral-Fehler: Abstand zur Wand
                lateral_error = self.target_dist - d_dist

                # Heading-Fehler: Wandgeometrie (Wand schräg?)
                # Wenn right_front näher als right → Wand läuft schräg nach vorn → links drehen.
                if np.isfinite(d_right_fwd):
                    heading_error = d_dist - d_right_fwd*self.cos50
                else:
                    heading_error = 0.0

                omega = self.K_lat * lateral_error + self.K_head * heading_error
                v = self.max_speed

                # Bei starkem Lenkeinschlag langsam fahren
                if abs(omega) > 1.0:
                    v *= 0.5

                linear  = v
                angular = omega
            else:
                # Fallback, falls doch keine Wandmessung
                linear  = 0.25
                angular = -0.25

        return linear, angular

