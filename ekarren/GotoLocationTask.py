#!/usr/bin/env python3
import numpy as np
from params import LidarMaxAngle, TaskState


ALIGN_90 = 0
DRIVE_90 = 1
ALIGN_0 = 2
DRIVE_0 = 3

class GotoLocationTask:
    def Init(self, navigator, params, retvals=None):
        
        self.nav = navigator                
        self.wantedDistX, self.wantedDistY = params
        
        # Basis-Vorwärtsgeschwindigkeit
        self.base_speed = 0.5

        self.state = ALIGN_0
        self.nav.SetWantedTheta(0.0)

        self.time = 0

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

        a = 5
        dist = self.GetSectorMin_deg(ranges, -a, a) 
        
        # Positionierung in x-Richtung
        if self.state == ALIGN_0:
            if self.nav.wantedThetaReached:
                self.sign = -1.0 if dist < self.wantedDistX else 1.0
                self.nav.SetDirection(0.0, self.sign*self.base_speed)
                self.state = DRIVE_0 
        elif self.state == DRIVE_0:
            if self.sign*(dist - self.wantedDistX) < 0:
                self.nav.SetWantedTheta(np.pi/2)
                self.state = ALIGN_90

        # Positionierung in y-Richtung
        elif self.state == ALIGN_90:
            if self.nav.wantedThetaReached:
                self.sign = -1.0 if dist < self.wantedDistY else 1.0
                self.nav.SetDirection(np.pi/2, self.sign*self.base_speed)
                self.state = DRIVE_90
        elif self.state == DRIVE_90:
            if self.sign*(dist - self.wantedDistY) < 0:
                self.state = ALIGN_0
                self.nav.ResetDirection()
                return TaskState.Ready, None
        else:
            print("[GotoLocationTask] ERROR illegal state")
            return TaskState.Error, None
            
        self.time += 1
        return TaskState.Running, None

