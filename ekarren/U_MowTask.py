#!/usr/bin/env python3
import numpy as np
import math
from params import LidarMaxAngle, TaskState, Ardumower
import helper as h

# Zustände
FOLLOW = 0
TURN   = 1
NONE   = 2

class U_MowTask:
    def Init(self, navigator, params, retvals=None):
        self.gamma_deg = 70
        gamma = math.radians(self.gamma_deg)
        self.cos_gamma = math.cos(gamma)   # winkel zwischen d_wall und d_wall45
        self.tan_gamma = math.tan(gamma)

        # Basis-Vorwärtsgeschwindigkeit
        self.base_speed = 0.5
        
        self.debugFlag = True
        self.nav = navigator
        
        
        # Maximal Winkelgeschwindigkeit
        self.max_angular = 1.5  # 0.5
        self.avoid_omega = 1.0

        # PD-Gewichte
        #self.K_lat  = 2.0  # 2.0  # lateral error (Abstand)
        #self.K_head = self.K_lat*0.5  # 1.0 # heading error (Wandorientierung)
        self.K_lat  = 1.0    # 0.5   # lateral error (Abstand)
        self.K_head = 0.1*0  # 0.3 1.0 # heading error (Wandorientierung)

        # Mähbereich
        self.startDist = 4.6
        self.endDist = 1.0
        self.laneDist = -0.2        
        self.forwardTurnDist = 1.5
        self.backwardTurnDist = 12.0
        self.wallAngle = np.radians(85)
                
        # Zielabstand zur rechten Wand [m]
        self.target_dist = self.startDist
        self.forward = True

        self.state = NONE
        #self.nav.SetWantedTheta(self.wallAngle)
        self.SetState(TURN)

        self.time = 0

        self.headErrSum = 0.0
        self.latErrSum = 0.0        
        
        self.turnCnt = 0
        self.turnCntMax = 5
        self.nav.SetMode(8)


    def GetStateText(self, newState):
        if newState==FOLLOW: return "FOLLOW"
        elif newState==TURN: return "TURN"
        return "Unknown"
    
    def SetState(self, newState):
        if newState != self.state:
            if self.debugFlag: print(f"State: {self.GetStateText(newState)}")
            self.state = newState

    # Step wird immer aufgerufen, wenn neue Lidar-Daten verfügbar sind.
    def Step(self, ranges):
        #r = np.array(ranges, dtype=float)
        assert ranges.size == 2*LidarMaxAngle

        d_zaun = 0
        angular = 0
        linear = 0

        # -------- Zustandsverhalten --------
        lateral_error = 0
        heading_error = 0
        omega = 0
        if self.state == TURN:
            print(f"{self.headErrSum=}  {self.latErrSum=}")
            self.headErrSum = 0.0
            self.latErrSum = 0.0      
            self.turnCnt += 1
            self.nav.SetVelocities(0.0, 0.0)
            if self.turnCnt >= self.turnCntMax:
                self.SetState(FOLLOW)

                
        elif self.state == FOLLOW:
            # --- Sektoren (Standard: 0° vorne, 90° links, 270° rechts) ---
            a = 5
            d_zaun   = h.GetSectorMedian(ranges,   0, 2*a) 
            d_wall   = h.GetSectorMin(ranges, -90-a,-90+a)  
            d_wall_gamma = h.GetSectorMin(ranges, -(180-self.gamma_deg)-a, -(180-self.gamma_deg)+a)     # Messung bei Gamma
            if Ardumower:
                # Lidarsensor virtuell um C virtuell nach vorne verschieben
                A = d_wall
                B = d_wall_gamma
                C = 0.5
                # m ist die Steigung der Geraden (=Wall)
                m = self.tan_gamma - A / (B*self.cos_gamma)
                cos_theta = 1.0 / math.sqrt(1.0 + m**2)
                d_wall = A * cos_theta
                d_wall_new = (A - C*m)*cos_theta

            if self.forward:
                s = 1.0
                # Lateral-Fehler: Abstand zur Wand
                lateral_error = self.target_dist - d_wall_new
                self.K_lat  = 0.35    # 0.5   # lateral error (Abstand)
                self.K_head = 0.60    # 0.3 1.0 # heading error (Wandorientierung)
            else:
                s = -1.0
                # Lateral-Fehler: Abstand zur Wand
                lateral_error = self.target_dist - d_wall
                self.K_lat  = 0.35    # 0.5   # lateral error (Abstand)
                self.K_head = 0.60    # 0.3 1.0 # heading error (Wandorientierung)

            # --- 5. Perfekt abgestimmte Regler-Werte (Kritische Dämpfung) ---
            #self.K_lat  = 1.0  
            #self.K_head = 1.5 

            # Wenn right_front näher als right → Wand läuft schräg nach vorn → links drehen.
            heading_error = C*m   #d_wall - d_wall_new
            self.headErrSum += float(heading_error)
            self.latErrSum += float(lateral_error)
            
            omega = s*self.K_lat*lateral_error + self.K_head*heading_error
            angular = omega
            if angular > self.max_angular: angular = self.max_angular
            elif angular < -self.max_angular: angular = -self.max_angular
            linear = s*self.base_speed

            if self.debugFlag:
                headErr = math.degrees(math.atan(m))
                print(f"{self.time:6d}: fwd={self.forward} {self.GetStateText(self.state)} d(0°)={d_zaun:.2f}  d(90°)={d_wall:.2f}  d(γ)={d_wall_gamma:.2f}  "
                f"latErr={lateral_error:5.2f}  headErr={headErr:5.1f}°  ang={angular:5.2f}  lin={linear:.2f}  tdist={self.target_dist:5.2f}")

            if self.target_dist < 1.0:
                #self.state = self.StateIdle
                self.nav.ResetDirection()
                self.nav.RvizPrint("Mowing completed")
                return TaskState.Ready, None

            if self.forward:
                if d_zaun < self.forwardTurnDist:   # 8
                    self.nav.SetVelocities(0.0, 0.0)
                    self.forward = False
                    self.target_dist += self.laneDist
                    self.turnCnt = 0
                    self.SetState(TURN)
            elif d_zaun > self.backwardTurnDist:  # 12.0:
                    self.nav.SetVelocities(0.0, 0.0)
                    self.forward = True
                    #self.target_dist += self.laneDist
                    self.turnCnt = 0
                    self.SetState(TURN)
                    
            if self.state == TURN:
                self.nav.SetVelocities(0.0, 0.0)
            else:
                self.nav.SetVelocities(angular, linear) 
            
        self.time += 1
        return TaskState.Running, None
