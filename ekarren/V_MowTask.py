#!/usr/bin/env python3
import numpy as np
from params import LidarMaxAngle, LidarRangeMax, TaskState


ALIGN           = 0
FORWARD         = 1
FORWARD_STOP    = 2
BACKWARD        = 3
BACKWARD_STOP   = 4
NONE            = 5

class V_MowTask:
    def Init(self, navigator, params, retvals=None):

        # Speed Control
        self.speedSign = 1    # foward: +1, backward -1
        self.speedCnt = 0
        self.speedCntMax = 10
        self.speedStep = 0.1    # m/s
        self.base_speed = self.speedCntMax*self.speedStep
        
        # Grashalm-Filter
        self.distReachedCnt = 0
        self.distReachedCntThreshold = 1
        
        self.nav = navigator
        self.debugFlag = True
        
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
        elif newState==FORWARD_STOP: return "FORWARD_STOP"
        elif newState==BACKWARD: return "BACKWARD"
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
        
    def NextLane(self, angle, sign):
        self.speedSign = sign
        self.speedCnt = 0
        self.distReachedCnt = 0
        self.nav.SetDirection(angle, 0.0)      #self.base_speed)

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
        assert ranges.size == 2*LidarMaxAngle

        a = 5
        distX = self.GetSectorMedian(ranges,   0, 2*a) 
        distY = self.GetSectorMedian(ranges,  90-a, 90+a)  
        
        if self.state == ALIGN:
            if self.nav.wantedThetaReached:
                self.NextLane(0.0, -1)
                self.SetState(BACKWARD)
                
        elif self.state == BACKWARD:
            self.IncSpeed();
            if distX > self.backwardTurnDist:
                self.distReachedCnt += 1
                if self.distReachedCnt >= self.distReachedCntThreshold:
                    self.DecSpeed();
                    self.SetState(BACKWARD_STOP)
            else:
                self.distReachedCnt = 0
                
        elif self.state == BACKWARD_STOP:
            done = self.DecSpeed();
            if done: 
                self.NextLane(self.laneAngle, 1)
                self.SetState(FORWARD)
                
        elif self.state == FORWARD:
            self.IncSpeed();
            if distX < self.forwardTurnDist:
                self.distReachedCnt += 1
                if self.distReachedCnt >= self.distReachedCntThreshold:
                    if distY < self.endDistY:
                        self.nav.ResetDirection()
                        return TaskState.Ready, None
                    self.DecSpeed();
                    self.SetState(FORWARD_STOP)
            else:
                self.distReachedCnt = 0
                
        elif self.state == FORWARD_STOP:
            done = self.DecSpeed();
            if done: 
                self.NextLane(0.0, -1)
                self.SetState(BACKWARD)

        if self.debugFlag:
            print(f"{self.time:6d}: {self.GetStateText(self.state)} {distX=:.2f}  {distY=:.2f}")

        self.time += 1
        return TaskState.Running, None

