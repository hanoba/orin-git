#!/usr/bin/env python3
import numpy as np
import math
from params import LidarMaxAngle, TaskState


SEARCH = 0
FOLLOW = 1
TURN   = 2
TURN2  = 3
CORNER = 4
NONE   = 5

class WallFollowerTask:
    def Init(self, node, params, retvals=None):
        
        self.cos45 = math.cos(45/180*math.pi)   # winkel zwischen d_wall und d_wall45

        # Basis-Vorwärtsgeschwindigkeit
        self.base_speed = 0.5
        
        self.debugFlag = True
        self.node = node
        self.subState = 0
        self.laneDist = -0.2        
        
        
        # Maximal Winkelgeschwindigkeit
        self.max_angular = 1.5  # 0.5
        self.avoid_omega = 1.0

        # PD-Gewichte
        #self.K_lat  = 2.0  # 2.0  # lateral error (Abstand)
        #self.K_head = self.K_lat*0.5  # 1.0 # heading error (Wandorientierung)
        self.K_lat  = 0.5  # 2.0 2.0  # lateral error (Abstand)
        self.K_head = 0.1*0  # 0.3 1.0 # heading error (Wandorientierung)

        # Mähbereich
        self.startDist = 5.6
        self.endDist = 1.0
        self.forwardTurnDist = 1.5
        self.backwardTurnDist = 12.0
                
        # Zielabstand zur rechten Wand [m]
        self.target_dist = self.startDist
        self.forward = True

        self.state = NONE
        self.SetState(FOLLOW)

        self.time = 0

    def GetStateText(self, newState):
        if newState==SEARCH: return "SEARCH"
        elif newState==FOLLOW: return "FOLLOW"
        elif newState==TURN: return "TURN"
        elif newState==TURN2: return "TURN2"
        elif newState==CORNER: return "CORNER"
        return "Unknown"
    
    def SetState(self, newState):
        if newState != self.state:
            if self.debugFlag: print(f"State: {self.GetStateText(newState)}")
            self.state = newState

    # Hilfsfunktion: Sektor mit Wraparound in Grad
    def GetSectorMin_deg(self, ranges_np, start_deg, end_deg):
        start_deg += LidarMaxAngle
        end_deg += LidarMaxAngle
        """Gibt Minimalradius in einen Bereich in [start_deg, end_deg) zurück."""
        if start_deg < end_deg: return np.nanmin(ranges_np[start_deg:end_deg]) 
        elif start_deg < end_deg: return self.maxDist_mm
        min1 = np.nanmin(ranges_np[start_deg:])
        min2 = np.nanmin(ranges_np[:end_deg])
        return min(min1, min2)  
        

    def Step(self, ranges):
        #r = np.array(ranges, dtype=float)
        assert ranges.size == 2*LidarMaxAngle

        # Inf / 0 als NaN behandeln
        ranges[~np.isfinite(ranges)] = np.nan
        ranges[ranges <= 0.0] = np.nan
        
        d_zaun = 0
        angular = 0
        linear = 0

        # -------- Zustandsverhalten --------
        lateral_error = 0
        heading_error = 0
        omega = 0
        if self.state == TURN:
            if self.node.wantedThetaReached:
                self.SetState(FOLLOW)
                
        elif self.state == FOLLOW:
            # --- Sektoren (Standard: 0° vorne, 90° links, 270° rechts) ---
            a = 5
            if self.forward:
                s = 1.0
                d_zaun   = self.GetSectorMin_deg(ranges,   0, 2*a) 
                d_wall   = self.GetSectorMin_deg(ranges, -90-a,-90+a)  
                d_wall45 = self.GetSectorMin_deg(ranges, -135-a, -135+a)     # Wandmessung um 45° versetzt
            else:
                s = -1.0
                #d_zaun = min(self.GetSectorMin_deg(ranges, 180-a,  180),
                #             self.GetSectorMin_deg(ranges,  -179, -179+a))
                d_zaun = self.GetSectorMin_deg(ranges,  -179, -179+2*a)
                d_wall     = self.GetSectorMin_deg(ranges,  90-a, 90+a)  
                d_wall45 = self.GetSectorMin_deg(ranges,    45-a, 45+a)  

            # Nur wenn Wandmessung vernünftig
            assert np.isfinite(d_wall)
            assert np.isfinite(d_wall45)
            
            # Lateral-Fehler: Abstand zur Wand
            lateral_error = self.target_dist - d_wall

            # Heading-Fehler: Wandgeometrie (Wand schräg?)
            # Wenn right_front näher als right → Wand läuft schräg nach vorn → links drehen.
            heading_error = d_wall - d_wall45*self.cos45

            omega = s*(self.K_lat * lateral_error - self.K_head * heading_error)
            angular = omega
            if angular > self.max_angular: angular = self.max_angular
            elif angular <- self.max_angular: angular = -self.max_angular
            linear = self.base_speed

            self.node.SetVelocities(angular, linear) 

            if self.debugFlag:
                print(f"{self.time:6d}: fwd={self.forward} {self.GetStateText(self.state)} d(0°)={d_zaun:.2f}  d(90°)={d_wall:.2f}  d(45°)={d_wall45:.2f}  "
                f"latErr={lateral_error:5.2f}  headErr={heading_error:5.2f}  ang={angular:5.2f}  lin={linear:.2f}  tdist={self.target_dist:5.2f}")

            if self.target_dist < 1.0:
                #self.state = self.StateIdle
                self.node.ResetDirection()
                self.node.RvizPrint("Mowing completed")
                return TaskState.Ready, None

            vLinear = 0.2
            if self.forward:
                #if d_zaun < 2.0 and abs(heading_error)<0.1 and False:
                if d_zaun < self.forwardTurnDist:   # 8
                    self.node.SetVelocities(0.0, 0.0)
                    self.forward = False
                    self.target_dist += self.laneDist
                    self.theta2 = self.node.theta - np.pi
                    self.node.SetWantedTheta(self.node.theta - np.radians(179), vLinear)
                    self.SetState(TURN)
            elif d_zaun > self.backwardTurnDist:  # 12.0:
                    self.node.SetVelocities(0.0, 0.0)
                    self.forward = True
                    self.target_dist += self.laneDist
                    self.theta2 = self.node.theta + np.pi
                    self.node.SetWantedTheta(self.node.theta + np.radians(179), vLinear)
                    self.SetState(TURN)
        self.time += 1
        return TaskState.Running, None

