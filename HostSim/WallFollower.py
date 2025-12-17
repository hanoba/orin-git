#!/usr/bin/env python3
import numpy as np
import math

lidarX=np.zeros(360*4)
lidarY=np.zeros(360*4)

class Lidar():
    def __init__(self, lidar, measPerDeg, backWheelDrive):
        self.frameCnt=-1
        self.last_step = -1
        self.dist = np.zeros(360*measPerDeg) + 9.0
        self.angles = np.zeros(360*measPerDeg) + 400.0
        self.lidar = lidar
        self.lidar.initialize()                   # wichtig
        self.lidar.add_point_cloud_data_to_frame()
        self.lidar.enable_visualization()
        self.debugCnt = 0
        self.debugCntMax = 10
        self.measPerDeg = measPerDeg
        self.totalPoints = 0
        self.angOffset = 180 if backWheelDrive else 0      # 0°=Front 180°=Back
        
    def Debug(self, text):
        if self.debugCnt < self.debugCntMax:
            print(f"[Lidar.Debug] {text}")
            
    def GetDistArray(self):
        self.debugCnt += 1
        
        # LiDAR-Frame holen
        frame = self.lidar.get_current_frame()
        if frame is None:
            return [], []

        new_step = frame["physics_step"]
        if new_step == self.last_step:
            return [], []
        self.last_step = new_step
        
        self.frameCnt += 1
        # skip first dummy frame
        if self.frameCnt == 0:
            return [], []
            
        global lidarX, lidarY
        pc = frame["point_cloud"].reshape(-1, 3)   # shape (N, 1, 3) – Punkte im Lidar/Robot-Frame

        #self.totalPoints += numPoints
        
        angMin =  99000
        angMax = -99000
        
        numPoints = len(pc[:, 0])
        for i in range(numPoints):
            x = pc[i, 0]
            y = pc[i, 1]

            ang = math.degrees(math.atan2(y, x))
            ang = (ang + self.angOffset) % 360.0      # Lidar 180° gedreht

            r = math.hypot(x, y)            # Entfernung            
            lidarX[self.totalPoints] = pc[i, 0]
            lidarY[self.totalPoints] = pc[i, 1]
            
            ### x = self.vz*pc[i, 0]
            ### y = self.vz*pc[i, 1]
            ### r = np.sqrt(x*x + y*y)
            ### ang = math.atan2(y, x)*180/math.pi
            ### if ang<0: ang += 360
            ### elif ang>360: ang -= 360
            
            self.angles[self.totalPoints] = ang
            self.dist[self.totalPoints] = r
            self.totalPoints += 1
            if ang < angMin: angMin = ang
            elif ang > angMax: angMax = ang
            #theta = int(round((ang)))
            #if theta<0: theta += 360*self.measPerDeg
            #self.dist[theta] = r
            #if theta < thetaMin: thetaMin = theta
            #elif theta > thetaMax: thetaMax = theta

        self.Debug(f"{numPoints=}  {angMin=:6.2f}  {angMax=:6.2f}")

        if self.totalPoints >= 360*self.measPerDeg:
            self.totalPoints = 0

            # 1. Die Indizes berechnen, die 'angles' sortieren würden
            idx = np.argsort(self.angles)

            # 2. Die Indizes auf beide Arrays anwenden
            return self.dist[idx], self.angles[idx]

        return [], []



SEARCH = 0
FOLLOW = 1
AVOID  = 2
ALIGN  = 3
NONE   = 4


class WallFollowerFinal:
    def __init__(self, world, target_dist=0.4, max_speed=0.5):
        # Zielabstand zur rechten Wand [m]
        self.target_dist = target_dist
        self.cos50 = math.cos(40/180*math.pi)   # winkel zwischen Wandabstand und right_front
        self.cos10 = math.cos( 0/180*math.pi)   # winkel zwischen Wandabstand und right 

        # Basis-Vorwärtsgeschwindigkeit
        self.max_speed = max_speed
        
        # Maximal Winkelgeschwindigkeit
        self.max_angular = 0.5
        self.avoid_omega = 1.0

        # PD-Gewichte
        self.K_lat  = 2.0  # 2.0  # lateral error (Abstand)
        self.K_head = self.K_lat*0.5  # 1.0 # heading error (Wandorientierung)

        # Schwellenwerte
        self.front_thresh = 1.5*self.target_dist  # 0.5   # Kollisionsschutz vorne
        self.far_thresh   = 2*self.target_dist  #  1.2    # ab hier gilt "Wand verloren"
        self.Init()
        self.world = world
        self.breakPointReached = False

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
    def GetSectorMin_deg(self, ranges_np, angles_np, start_deg, end_deg):
        """Gibt Minimalradius in einen Bereich in [start_deg, end_deg) zurück."""
        if start_deg <= end_deg:
            indices = np.where((angles_np >= start_deg) & (angles_np <= end_deg))[0]
        else:
            indices = np.where((angles_np >= start_deg) | (angles_np <= end_deg))[0]
        return np.nanmin(ranges_np[indices])  
        
        #measPerDeg = ranges_np.size // 360      # Messwerte pro Grad
        #start_deg %= 360
        #end_deg   %= 360
        #start_deg *= measPerDeg
        #end_deg *= measPerDeg
        #if start_deg <= end_deg:
        #    sector = ranges_np[start_deg:end_deg]
        #else:
        #    sector = np.concatenate((ranges_np[start_deg:], ranges_np[:end_deg]))
        ##return np.nanmean(sector) if np.any(~np.isnan(sector)) else np.inf
        #return np.nanmin(sector) if np.any(~np.isnan(sector)) else np.inf

    def step(self, ranges, angles):
        #r = np.array(ranges, dtype=float)
        assert ranges.size % 360 == 0

        # Inf / 0 als NaN behandeln
        ranges[~np.isfinite(ranges)] = np.nan
        ranges[ranges <= 0.0] = np.nan
        
        ## Falls der Scan nicht 360 Werte hat, auf 360 „normalisieren“
        ## (vereinfachend: bei den meisten 360°-Scannern ist das der Fall,
        ##  sonst müsste man über angle_min/angle_increment gehen).
        #if ranges.size != 360:
        #    print(f"ERROR {ranges.size=}")
        #    # grobe, aber robuste Resampling-Lösung
        #    idx = np.linspace(0, ranges.size - 1, 360).astype(int)
        #    ranges = ranges[idx]

        # --- Sektoren (Standard: 0° vorne, 90° links, 270° rechts) ---
        d_front     = self.GetSectorMin_deg(ranges, angles, 350, 10)   #   0° +/- 10°
        d_right     = self.GetSectorMin_deg(ranges, angles, 270, 290)  # 280° +/- 10° rechts seitlich
        d_right_fwd = self.GetSectorMin_deg(ranges, angles, 310, 330)  # 320° +/- 10° rechts vorne

        d_dist = d_right*self.cos10

        # -------- Zustandswahl --------
        # 1) Hindernis vorne: immer AVOID
        if d_front < self.front_thresh:
            self.SetState(AVOID)
        else:
            # Wand verloren? → SEARCH
            th = self.far_thresh
            if self.state == FOLLOW: th=th*1.5    #1.2  # Hysterese
            if self.state != ALIGN:
                if d_dist > th or np.isinf(d_right):
                    self.SetState(SEARCH)
                else:
                    self.SetState(FOLLOW)

        # -------- Zustandsverhalten --------
        lateral_error = 0
        heading_error = 0
        omega = 0
        if self.state == AVOID:
            # Hindernis vorne → stehen bleiben, links drehen
            linear  = 0.0
            angular = self.avoid_omega

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
                angular = -self.max_angular
                linear  = self.max_speed
            else:
                angular = 0    # -0.5   
                linear = self.max_speed
            #angular = -0.2   # -0.25
            #if d_right_fwd < d_right: angular = 0.1  #HB

        elif self.state == ALIGN:
            # Roboter ungefähr parallel zur Wand ausrichten
            linear = 0
            angular = self.max_angular
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
                angular = omega
                #if angular > self.max_angular: angular = self.max_angular
                #elif angular <- self.max_angular: angular = -self.max_angular
                v = self.max_speed

                # Bei starkem Lenkeinschlag langsam fahren
                if abs(omega) > 0.2:
                    v *= 0.2
                linear  = v

            else:
                # Fallback, falls doch keine Wandmessung
                linear  = 0.25
                angular = -0.25


        time = self.world.current_time
        if not self.breakPointReached:
            from omni.isaac.core.prims import XFormPrim
            prim = XFormPrim("/World/eKarren/chassis_link")
            pos, quat = prim.get_world_pose()
            posX = pos[0]
            posY = pos[1]
            from scipy.spatial.transform import Rotation as R
            # (w,x,y,z) → (x,y,z,w)
            quat_scipy = [quat[1], quat[2], quat[3], quat[0]]
            r = R.from_quat(quat_scipy)   # (x, y, z, w)
            yaw = r.as_euler("xyz", degrees=True)[2]
            print(f"{time:6.2f}: {self.GetStateText(self.state)} {d_front=:.2f}  {d_dist=:.2f}  {d_right_fwd=:.2f}  "
                f"{lateral_error=:5.2f}  {heading_error=:5.2f}  {angular=:5.2f}  {linear=:.2f}  "
                f"{posX=:5.2f} {posY=:5.2f} {yaw=:4.1f}°"
            )

        if time > 99.11:
            if not self.breakPointReached:
                np.savetxt("/DriveX/GitHub/orin-git/HostSim/ranges.txt", ranges, fmt="%.3f")
                np.savetxt("/DriveX/GitHub/orin-git/HostSim/angles.txt", angles, fmt="%.3f")
                np.savetxt("/DriveX/GitHub/orin-git/HostSim/lidarX.txt", lidarX, fmt="%8.3f")
                np.savetxt("/DriveX/GitHub/orin-git/HostSim/lidarY.txt", lidarY, fmt="%8.3f")
                self.breakPointReached = True
                print("Breakpoint reached")
            return 0, 0
        return linear, angular

