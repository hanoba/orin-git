#!/usr/bin/env python3
import numpy as np
from params import LidarMaxAngle, LidarRangeMax, TaskState


ALIGN           = 0
FORWARD_START   = 1
FORWARD         = 2
FORWARD_STOP    = 3
BACKWARD_START  = 4
BACKWARD        = 5
BACKWARD_STOP   = 6
NONE            = 7

class V_MowTask:
    def Init(self, navigator, params, retvals=None):

        # Speed Control
        self.speedSign = 1    # foward: +1, backward -1
        self.speedCnt = 0
        self.speedCntMax = 10
        self.speedStep = 0.05    # m/s
        self.base_speed = self.speedCntMax*self.speedStep
        
        self.nav = navigator
        self.debugFlag = True
        
        # Maximal Winkelgeschwindigkeit
        self.max_angular = 1.5  # 0.5

        # Mähbereich
        self.endDistY = 2.0
        self.backwardTurnDist = 7.0
        self.forwardTurnDist = 4.0

        self.laneAngle = 0.2 / (self.backwardTurnDist - self.forwardTurnDist)
                
        self.state = NONE
        self.SetState(ALIGN)

        self.time = 0
        self.nav.SetWantedTheta(0.0)

    def GetStateText(self, newState):
        if newState==ALIGN: return "ALIGN"
        elif newState==FORWARD: return "FORWARD"
        elif newState==FORWARD_START: return "FORWARD_START"
        elif newState==FORWARD_STOP: return "FORWARD_STOP"
        elif newState==BACKWARD: return "BACKWARD"
        elif newState==BACKWARD_START: return "BACKWARD_START"
        elif newState==BACKWARD_STOP: return "BACKWARD_STOP"
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
            
        # Sicherheitscheck: Gibt es überhaupt N Werte?
        if len(new_ranges) < N:
            # Wenn nicht genug Daten da sind, gib den sicheren Maximalwert 
            return LidarRangeMax
        sorted_ranges = np.sort(new_ranges)
        return sorted_ranges[N-1]
        
    def ResetSpeed(self, sign):
        self.speedSign = sign
        self.speedCnt = 0
        
    def IncSpeed(self):
        self.speedCnt += 1
        done = self.speedCnt >= self.speedCntMax
        if done:
            self.speedCnt = self.speedCntMax
        self.nav.SetLinearVelocity(self.speedCnt*self.speedStep*self.speedSign)
        return done
        
    def DecSpeed(self):
        self.speedCnt -= 1
        done = self.speedCnt <= 0
        if done:
            self.speedCnt = 0
            self.nav.ResetDirection()
        else:
            self.nav.SetLinearVelocity(self.speedCnt*self.speedStep*self.speedSign)
        return done

    def Step(self, ranges):
        #r = np.array(ranges, dtype=float)
        assert ranges.size == 2*LidarMaxAngle

        # Inf / 0 als NaN behandeln
        #ranges[~np.isfinite(ranges)] = np.nan
        #ranges[ranges <= 0.0] = np.nan
        
        a = 5
        distX = self.GetSectorMedian(ranges,   0, 2*a) 
        distY = self.GetSectorMedian(ranges,  90-a, 90+a)  
        
        if self.state == ALIGN:
            if self.nav.wantedThetaReached:
                self.ResetSpeed(-1)
                self.nav.SetDirection(0.0, 0.0)     #-self.base_speed)
                self.SetState(BACKWARD_START)
                
        elif self.state == BACKWARD_START:
            done = self.IncSpeed();
            if done: 
                self.SetState(BACKWARD)
                
        elif self.state == BACKWARD:
            if distX > self.backwardTurnDist:
                self.DecSpeed();
                self.SetState(BACKWARD_STOP)
                
        elif self.state == BACKWARD_STOP:
            done = self.DecSpeed();
            if done: 
                self.ResetSpeed(1)
                self.nav.SetDirection(self.laneAngle, 0.0)      #self.base_speed)
                self.SetState(FORWARD_START)
                
        elif self.state == FORWARD_START:
            done = self.IncSpeed();
            if done: 
                self.SetState(FORWARD)
                
        elif self.state == FORWARD:
            if distX < self.forwardTurnDist:
                if distY < self.endDistY:
                    self.nav.ResetDirection()
                    return TaskState.Ready, None
                self.DecSpeed();
                self.SetState(FORWARD_STOP)
                
        elif self.state == FORWARD_STOP:
            done = self.DecSpeed();
            if done: 
                self.ResetSpeed(-1)
                self.nav.SetDirection(0.0, 0.0)      #self.base_speed)
                self.SetState(BACKWARD_START)

        if self.debugFlag:
            print(f"{self.time:6d}: {self.GetStateText(self.state)} {distX=:.2f}  {distY=:.2f}")

        self.time += 1
        return TaskState.Running, None

