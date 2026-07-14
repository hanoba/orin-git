#!/usr/bin/env python3
import numpy as np
from params import LidarMaxAngle, LidarRangeMax, TaskState


ALIGN = 0
FORWARD = 1
BACKWARD = 2
NONE   = 3

class V_MowTask:
    def Init(self, navigator, params, retvals=None):

        # Basis-Vorwärtsgeschwindigkeit
        self.base_speed = 0.5
        
        self.nav = navigator
        self.debugFlag = True
        
        # Maximal Winkelgeschwindigkeit
        self.max_angular = 1.5  # 0.5

        # Mähbereich
        self.endDistY = 2.0
        self.forwardTurnDist = 7.0
        self.backwardTurnDist = 4.0

        self.laneAngle = 0.2 / (self.forwardTurnDist - self.backwardTurnDist)
                
        self.state = NONE
        self.SetState(ALIGN)

        self.time = 0
        self.nav.SetWantedTheta(0.0)

    def GetStateText(self, newState):
        if newState==ALIGN: return "ALIGN"
        elif newState==FORWARD: return "FORWARD"
        elif newState==BACKWARD: return "BACKWARD"
        return "Unknown"
    
    def SetState(self, newState):
        if newState != self.state:
            if self.debugFlag: print(f"State: {self.GetStateText(newState)}")
            self.state = newState

    # Hilfsfunktion: Sektor mit Wraparound in Grad
    # Gibt Minimalradius in einen Bereich [start_deg, end_deg) zurück.
    def GetSectorMin(self, ranges_np, start_deg, end_deg):
        start_deg += LidarMaxAngle
        end_deg += LidarMaxAngle
        if start_deg < end_deg: return np.nanmin(ranges_np[start_deg:end_deg]) 
        #elif start_deg < end_deg: return self.maxDist_mm
        min1 = np.nanmin(ranges_np[start_deg:])
        min2 = np.nanmin(ranges_np[:end_deg])
        return min(min1, min2)  

    # Hilfsfunktion: Sektor mit Wraparound in Grad
    # Gibt den Median in einen Bereich [start_deg, end_deg) zurück.
    # Kann so interpretiert werden, dass die Hälfte der Werte kleiner oder gleich dem Median sind.
    # Das sollte helfen einzelne Grashalme zu ignorieren.
    def GetSectorMedian(self, ranges_np, start_deg, end_deg):
        start_deg += LidarMaxAngle
        end_deg += LidarMaxAngle
        if start_deg < end_deg: return np.median(ranges_np[start_deg:end_deg]) 
        #elif start_deg < end_deg: return self.maxDist_mm
        new_ranges = np.append(ranges_np[start_deg:], ranges_np[:end_deg])
        return np.median(new_ranges)

    # Hilfsfunktion: Sektor mit Wraparound in Grad
    # Berechnet den Minimalwert, der von N Range-Werten nicht überschritten wird.
    # Das sollte helfen einzelne Grashalme zu ignorieren.
    def GetRobustSectorMin(self, ranges_np, start_deg, end_deg, N):
        start_deg += LidarMaxAngle
        end_deg += LidarMaxAngle
        if start_deg < end_deg: 
            new_ranges = ranges_np[start_deg:end_deg]
        else:
            new_ranges = np.append(ranges_np[start_deg:], ranges_np[:end_deg])
            
        # Sicherheitscheck: Gibt es überhaupt N gültige Werte?
        if len(valid_ranges) < N:
            # Wenn nicht genug Daten da sind, gib den sicheren Maximalwert 
            return LidarRangeMax
        sorted_ranges = np.sort(new_ranges)
        return sorted_ranges[N-1]
        

    def Step(self, ranges):
        #r = np.array(ranges, dtype=float)
        assert ranges.size == 2*LidarMaxAngle

        # Inf / 0 als NaN behandeln
        ranges[~np.isfinite(ranges)] = np.nan
        ranges[ranges <= 0.0] = np.nan
        
        a = 5
        distX = self.GetSectorMedian(ranges,   0, 2*a) 
        distY = self.GetSectorMedian(ranges,  90-a, 90+a)  
        
        if self.state == ALIGN:
            if self.nav.wantedThetaReached:
                self.nav.SetDirection(0.0, -self.base_speed)
                self.SetState(BACKWARD)
                
        elif self.state == BACKWARD:
            # --- Sektoren (Standard: 0° vorne, 90° links, 270° rechts) ---
            if distX > self.forwardTurnDist:
                self.nav.SetDirection(self.laneAngle, self.base_speed)
                self.SetState(FORWARD)
        elif self.state == FORWARD:
            if distX < self.backwardTurnDist:
                if distY < self.endDistY:
                    self.nav.ResetDirection()
                    return TaskState.Ready, None
                self.nav.SetDirection(0.0, -self.base_speed)
                self.SetState(BACKWARD)

        if self.debugFlag:
            print(f"{self.time:6d}: {self.GetStateText(self.state)} {distX=:.2f}  {distY=:.2f}")

        self.time += 1
        return TaskState.Running, None

