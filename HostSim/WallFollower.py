#!/usr/bin/env python3
import numpy as np
import math


SEARCH = 0
FOLLOW = 1
AVOID  = 2
ALIGN  = 3
CORNER = 4
NONE   = 5

class WallFollower:
    def __init__(self, target_dist=0.4, base_speed=0.5):
        # Zielabstand zur rechten Wand [m]
        self.target_dist = target_dist
        self.cos50 = math.cos(40/180*math.pi)   # winkel zwischen Wandabstand und right_front
        self.cos10 = math.cos( 0/180*math.pi)   # winkel zwischen Wandabstand und right 

        # Basis-Vorwärtsgeschwindigkeit
        self.base_speed = base_speed
        
        # Paramters for corner detection
        self.corner_thresh =9.0    # Threshold for corner detection in meter
        self.corner_delay = 30      # Delay until turning right
        self.corner_cnt = 0         # delay counter
        self.corner_radius = 1.0    # corner curve radius 

        
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
        #self.breakPointReached = False
        self.time = 0

    def Init(self):
        print("Final WallFollower started (StateMachine + PD).")
        self.state = NONE
        self.SetState(SEARCH)

    def GetStateText(self, newState):
        if newState==SEARCH: return "SEARCH"
        elif newState==FOLLOW: return "FOLLOW"
        elif newState==AVOID: return "AVOID"
        elif newState==ALIGN: return "ALIGN"
        elif newState==CORNER: return "CORNER"
        return "Unknown"
    
    def SetState(self, newState):
        if newState != self.state:
            print(f"State: {self.GetStateText(newState)}")
            self.state = newState

    # Hilfsfunktion: Sektor mit Wraparound in Grad
    def GetSectorMin_deg(self, ranges_np, start_deg, end_deg):
        start_deg += 120
        end_deg += 120
        """Gibt Minimalradius in einen Bereich in [start_deg, end_deg) zurück."""
        if start_deg < end_deg: return np.nanmin(ranges_np[start_deg:end_deg]) 
        elif start_deg < end_deg: return self.maxDist_mm
        min1 = np.nanmin(ranges_np[start_deg:])
        min2 = np.nanmin(ranges_np[:end_deg])
        return min(min1, min2)  
        

    def step(self, ranges):
        #r = np.array(ranges, dtype=float)
        from Lidar import LidarMaxAngle
        assert ranges.size == 2*LidarMaxAngle

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
        d_front     = self.GetSectorMin_deg(ranges, -10, 10)   #   0° +/- 10°
        d_right     = self.GetSectorMin_deg(ranges, -90,-70)   # 280° +/- 10° rechts seitlich
        d_right_fwd = self.GetSectorMin_deg(ranges, -50,-30)   # 320° +/- 10° rechts vorne

        d_dist = d_right*self.cos10

        # -------- Zustandswahl --------
        # 1) Hindernis vorne: immer AVOID
        if d_front < self.front_thresh:
            self.SetState(AVOID)
        else:
            # Wand verloren? → SEARCH
            th = self.far_thresh
            if self.state == FOLLOW: 
                th=th*1.5    #1.2  # Hysterese
                if d_right_fwd >= self.corner_thresh:
                    self.SetState(CORNER)
                    self.corner_cnt = 0
            if self.state != ALIGN and self.state != CORNER:
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
                linear  = self.base_speed
            else:
                angular = 0    # -0.5   
                linear = self.base_speed
            #angular = -0.2   # -0.25
            #if d_right_fwd < d_right: angular = 0.1  #HB

        elif self.state == ALIGN:
            # Roboter ungefähr parallel zur Wand ausrichten
            linear = 0
            angular = self.max_angular
            if d_front > d_right_fwd and d_right_fwd > d_right:
                self.SetState(FOLLOW)

        elif self.state == CORNER:
            self.corner_cnt += 1
            linear = self.base_speed
            angular = 0.0 if self.corner_cnt < self.corner_delay else -linear/self.corner_radius
            if d_right_fwd < 2 * self.target_dist:
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
                v = self.base_speed

                # Bei starkem Lenkeinschlag langsam fahren
                #if abs(omega) > 0.2:
                #    v *= 0.2
                linear  = v

            else:
                # Fallback, falls doch keine Wandmessung
                linear  = 0.25
                angular = -0.25


        #time = self.world.current_time
        #if not self.breakPointReached:
        #    from omni.isaac.core.prims import XFormPrim
        #    prim = XFormPrim("/World/eKarren/chassis_link")
        #    pos, quat = prim.get_world_pose()
        #    posX = pos[0]
        #    posY = pos[1]
        #    from scipy.spatial.transform import Rotation as R
        #    # (w,x,y,z) → (x,y,z,w)
        #    quat_scipy = [quat[1], quat[2], quat[3], quat[0]]
        #    r = R.from_quat(quat_scipy)   # (x, y, z, w)
        #    yaw = r.as_euler("xyz", degrees=True)[2]
        #    print(f"{time:6.2f}: {self.GetStateText(self.state)} {d_front=:.2f}  {d_dist=:.2f}  {d_right_fwd=:.2f}  "
        #        f"{lateral_error=:5.2f}  {heading_error=:5.2f}  {angular=:5.2f}  {linear=:.2f}  "
        #        f"{posX=:5.2f} {posY=:5.2f} {yaw=:4.1f}°"
        #    )
        print(f"{self.time:6d}: {self.GetStateText(self.state)} d(0°)={d_front:.2f}  d(90°)={d_dist:.2f}  d(40°)={d_right_fwd:.2f}  "
            f"latErr={lateral_error:5.2f}  headErr={heading_error:5.2f}  ang={angular:5.2f}  lin={linear:.2f}  ")
        self.time += 1
        #if time > 99.11 and False:
        #    if not self.breakPointReached:
        #        np.savetxt("/DriveX/GitHub/orin-git/HostSim/ranges.txt", ranges, fmt="%.3f")
        #        np.savetxt("/DriveX/GitHub/orin-git/HostSim/angles.txt", angles, fmt="%.3f")
        #        np.savetxt("/DriveX/GitHub/orin-git/HostSim/lidarX.txt", lidarX, fmt="%8.3f")
        #        np.savetxt("/DriveX/GitHub/orin-git/HostSim/lidarY.txt", lidarY, fmt="%8.3f")
        #        self.breakPointReached = True
        #        print("Breakpoint reached")
        #    return 0, 0
        return linear, angular


if __name__ == "__main__":
    import sys
    from eKarrenLib import eKarren
    from pyqtgraph.Qt import QtWidgets
    from Lidar import LidarApp
    
    bot = eKarren(debug=True)
    follower = WallFollower(target_dist=2.00, base_speed=0.5)
    
    def ProcessLidarData(angles, dist):
        vLinear, omega = follower.step(dist)
        bot.SetSpeed(vLinear, omega)
        return [0], [0]

    app = QtWidgets.QApplication(sys.argv)
    window = LidarApp(ProcessLidarData)
    sys.exit(app.exec_())
    
