#!/usr/bin/env python3
import numpy as np
import math

class Lidar():
    def __init__(self, lidar, measPerDeg):
        self.frameCnt=-1
        self.last_step = -1
        self.dist = np.zeros(360*measPerDeg)
        self.lidar = lidar
        self.lidar.initialize()                   # wichtig
        self.lidar.add_point_cloud_data_to_frame()
        self.lidar.enable_visualization()
        self.debugCnt = 0
        self.debugCntMax = 10
        self.measPerDeg = measPerDeg
        self.totalPoints = 0
        
    def Debug(self, text):
        if self.debugCnt < self.debugCntMax:
            print(f"[Lidar.Debug] {text}")
            
    def GetDistArray(self):
        self.debugCnt += 1
        
        # LiDAR-Frame holen
        frame = self.lidar.get_current_frame()
        if frame is None:
            return []

        new_step = frame["physics_step"]
        if new_step == self.last_step:
            return []
        self.last_step = new_step
        
        self.frameCnt += 1
        # skip first dummy frame
        if self.frameCnt == 0:
            return []
            
        pc = frame["point_cloud"].reshape(-1, 3)   # shape (N, 1, 3) – Punkte im Lidar/Robot-Frame

        numPoints = len(pc[:, 0])
        self.totalPoints += numPoints
        
        thetaMin =  99000
        thetaMax = -99000
        
        for i in range(numPoints):
            x = pc[i, 0]
            y = pc[i, 1]
            r = np.sqrt(x*x + y*y)
            theta = int(round((self.measPerDeg*math.atan2(y, x)*180/math.pi)))
            if theta<0: theta += 360*self.measPerDeg
            self.dist[theta] = r
            if theta < thetaMin: thetaMin = theta
            elif theta > thetaMax: thetaMax = theta

        self.Debug(f"{numPoints=}  {thetaMin=}  {thetaMax=}")

        if self.totalPoints >= 360*self.measPerDeg:
            self.totalPoints = 0
            return self.dist

        return []



SEARCH = 0
FOLLOW = 1
AVOID  = 2
ALIGN  = 3
NONE   = 4


class WallFollowerFinal:
    def __init__(self, target_dist=0.4, max_speed=0.35):
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
        self.front_thresh = 0.6  # 0.5   # Kollisionsschutz vorne
        self.far_thresh   = 2*self.target_dist  #  1.2    # ab hier gilt "Wand verloren"
        self.Init()

    def Init(self):
        print("Final WallFollower started (StateMachine + PD).")
        self.state = NONE
        self.SetState(SEARCH)

    def GetStateText(self, newState):
        if newState==SEARCH: return "SEARCH"
        elif newState==FOLLOW: return "FOLLOW"
        elif newState==AVOID: return "AVOID"
        elif newState==ALIGN: return "ALIGN"
        return "Unknown"
    
    def SetState(self, newState):
        if newState != self.state:
            print(f"State: {self.GetStateText(newState)}")
            self.state = newState

    # Hilfsfunktion: Sektor mit Wraparound in Grad
    def get_sector_deg(self, ranges_np, start_deg, end_deg):
        """Gibt Minimalradius in einen Bereich in [start_deg, end_deg) zurück."""
        measPerDeg = ranges_np.size // 360      # Messwerte pro Grad
        start_deg %= 360
        end_deg   %= 360
        start_deg *= measPerDeg
        end_deg *= measPerDeg
        if start_deg <= end_deg:
            sector = ranges_np[start_deg:end_deg]
        else:
            sector = np.concatenate((ranges_np[start_deg:], ranges_np[:end_deg]))
        #return np.nanmean(sector) if np.any(~np.isnan(sector)) else np.inf
        return np.nanmin(sector) if np.any(~np.isnan(sector)) else np.inf

    def step(self, ranges):
        r = np.array(ranges, dtype=float)
        assert r.size % 360 == 0

        # Inf / 0 als NaN behandeln
        r[~np.isfinite(r)] = np.nan
        r[r <= 0.0] = np.nan
        
        ## Falls der Scan nicht 360 Werte hat, auf 360 „normalisieren“
        ## (vereinfachend: bei den meisten 360°-Scannern ist das der Fall,
        ##  sonst müsste man über angle_min/angle_increment gehen).
        #if r.size != 360:
        #    print(f"ERROR {r.size=}")
        #    # grobe, aber robuste Resampling-Lösung
        #    idx = np.linspace(0, r.size - 1, 360).astype(int)
        #    r = r[idx]

        # --- Sektoren (Standard: 0° vorne, 90° links, 270° rechts) ---
        d_front     = self.get_sector_deg(r, 350, 10)   #   0° +/- 10°
        d_right     = self.get_sector_deg(r, 270, 290)  # 280° +/- 10° rechts seitlich
        d_right_fwd = self.get_sector_deg(r, 310, 330)  # 320° +/- 10° rechts vorne

        d_dist = d_right*self.cos10
        print(f"{self.GetStateText(self.state)} {d_front=:.2f}  {d_dist=:.2f}  {d_right_fwd=:.2f}")

        # -------- Zustandswahl --------
        # 1) Hindernis vorne: immer AVOID
        if d_front < self.front_thresh:
            self.SetState(AVOID)
        else:
            # Wand verloren? → SEARCH
            th = self.far_thresh
            if self.state == FOLLOW: th=th*1.2  # Hysterese
            if self.state != ALIGN:
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
            # Wand suchen: schnell auf Wand zufahren
            # sanfte Rechtskurve, damit man nicht ewig parallel fährt,
            # aber auch keine engen Kreise.
            d_min = min(d_front, d_right_fwd, d_right)
            if d_min < self.far_thresh: 
                self.SetState(ALIGN)
                linear = 0
                angular = 0
            elif d_front > d_right_fwd: 
                angular = -0.5
                linear  = 0.5
            else:
                angular = 0    # -0.5   
                linear = 0.5
            #angular = -0.2   # -0.25
            #if d_right_fwd < d_right: angular = 0.1  #HB

        elif self.state == ALIGN:
            # Roboter ungefähr parallel zur Wand ausrichten
            linear = 0
            angular = 0.5
            if d_front > d_right_fwd and d_right_fwd > d_right:
                self.SetState(FOLLOW)

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




class WallFollowerTest:
    def __init__(self, target_dist=0.5, max_speed=0.35):
        # Zielabstand zur rechten Wand [m]
        self.target_dist = target_dist
        self.cos50 = math.cos(40/180*math.pi)   # winkel zwischen Wandabstand und right_front
        self.cos10 = math.cos( 0/180*math.pi)   # winkel zwischen Wandabstand und right 

        # Basis-Vorwärtsgeschwindigkeit
        self.max_speed = max_speed

        # PD-Gewichte
        self.K_lat  = 4.0  # 2.0  # lateral error (Abstand)
        self.K_head = 4.0  # 1.0 # heading error (Wandorientierung)

        # Schwellenwerte
        self.front_thresh = 0.5   # Kollisionsschutz vorne
        self.far_thresh   = 2*self.target_dist  #  1.2    # ab hier gilt "Wand verloren"
        self.Init()

    def Init(self):
        print("Final WallFollower started (StateMachine + PD).")
        self.state = NONE
        self.SetState(SEARCH)

    def GetStateText(self, newState):
        if newState==SEARCH: return "SEARCH"
        elif newState==FOLLOW: return "FOLLOW"
        elif newState==AVOID: return "AVOID"
        elif newState==ALIGN: return "ALIGN"
        return "Unknown"
    
    def SetState(self, newState):
        if newState != self.state:
            print(f"State: {self.GetStateText(newState)}")
            self.state = newState

    # Hilfsfunktion: Sektor mit Wraparound in Grad
    def get_sector_deg(self, ranges_np, start_deg, end_deg):
        """Gibt Minimalradius in einen Bereich in [start_deg, end_deg) zurück."""
        measPerDeg = ranges_np.size // 360      # Messwerte pro Grad
        start_deg %= 360
        end_deg   %= 360
        start_deg *= measPerDeg
        end_deg *= measPerDeg
        if start_deg <= end_deg:
            sector = ranges_np[start_deg:end_deg]
        else:
            sector = np.concatenate((ranges_np[start_deg:], ranges_np[:end_deg]))
        #return np.nanmean(sector) if np.any(~np.isnan(sector)) else np.inf
        return np.nanmin(sector) if np.any(~np.isnan(sector)) else np.inf

    def step(self, ranges):
        r = np.array(ranges, dtype=float)
        assert r.size % 360 == 0

        # Inf / 0 als NaN behandeln
        r[~np.isfinite(r)] = np.nan
        r[r <= 0.0] = np.nan

        # Falls der Scan nicht 360 Werte hat, auf 360 „normalisieren“
        # (vereinfachend: bei den meisten 360°-Scannern ist das der Fall,
        #  sonst müsste man über angle_min/angle_increment gehen).
        #if r.size != 360:
        #    print(f"ERROR {r.size=}")
        #    # grobe, aber robuste Resampling-Lösung
        #    idx = np.linspace(0, r.size - 1, 360).astype(int)
        #    r = r[idx]
        

        # --- Sektoren (Standard: 0° vorne, 90° links, 270° rechts) ---
        d_front     = self.get_sector_deg(r, 350, 10)   #   0° +/- 10°
        d_right     = self.get_sector_deg(r, 270, 290)  # 290° +/- 10° rechts seitlich
        d_right_fwd = self.get_sector_deg(r, 310, 330)  # 320° +/- 10° rechts vorne

        d_dist = d_right*self.cos10
        print(f"{self.GetStateText(self.state)} {d_front=:.2f}  {d_dist=:.2f}  {d_right_fwd=:.2f}")

        # -------- Zustandswahl --------
        # 1) Hindernis vorne: immer AVOID
        if d_front < self.front_thresh:
            self.SetState(AVOID)
        else:
            # Wand verloren? → SEARCH
            th = self.far_thresh
            #if self.state == FOLLOW: th=th*1.2  # Hysterese
            #if self.state != ALIGN:
            #    if d_dist > th or np.isinf(d_right):
            #        self.SetState(SEARCH)
            #    else:
            #        self.SetState(FOLLOW)

        # -------- Zustandsverhalten --------
        if self.state == AVOID:
            # Hindernis vorne → stehen bleiben, links drehen
            linear  = 0.0
            angular = 1.0

        elif self.state == SEARCH:
            # Wand suchen: schnell auf Wand zufahren
            # sanfte Rechtskurve, damit man nicht ewig parallel fährt,
            # aber auch keine engen Kreise.
            d_min = min(d_front, d_right_fwd, d_right)
            if d_min < self.far_thresh: 
                self.SetState(ALIGN)
                linear = 0
                angular = 0
            elif d_front > d_right_fwd: 
                angular = -0.5
                linear  = 0.5
            else:
                angular = 0    # -0.5   
                linear = 0.5
            #angular = -0.2   # -0.25
            #if d_right_fwd < d_right: angular = 0.1  #HB
            if d_dist < self.far_thresh and not np.isinf(d_right):
                self.SetState(ALIGN)

        elif self.state == ALIGN:
            # Roboter ungefähr parallel zur Wand ausrichten
            linear = 0
            angular = 0.5
            if d_front > d_right_fwd and d_right_fwd > d_right:
                self.SetState(FOLLOW)

        elif self.state == FOLLOW:
            # Nur wenn Wandmessung vernünftig
            if d_right < 10:
                # Lateral-Fehler: Abstand zur Wand
                lateral_error = self.target_dist - d_dist

                # Heading-Fehler: Wandgeometrie (Wand schräg?)
                # Wenn right_front näher als right → Wand läuft schräg nach vorn → links drehen.
                if d_right_fwd < 10:
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

                if d_dist > 1.2*self.far_thresh or d_right > 10:
                    self.SetState(SEARCH)
                #else:
                #    self.SetState(FOLLOW)

        return linear, angular

