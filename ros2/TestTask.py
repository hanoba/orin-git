import numpy as np
import math
import params
import json
import os
import time
from params import TaskState

# Task zum Testen des E-Karrens.

# Folgende Tests werden nacheinander ausgeführt:
# 1) Messung der Winkelgeschwindigkeit (wenn OmegaMeasFlag=True)
# 2) Ausrichtung des E-Karrens auf WantedYaw_deg
# 3) Fahrt in diese Richtung bis ein Hindernis erreicht wird (wenn GotoWallFlag=True)
#    Während der Fahrt wird vLinear gemessen

# Default parameter
OmegaMeasFlag = False
OmegaMeasDuration = 3.0
OmegaValue = 1.0
WantedYaw_deg = -90.0
GotoWallFlag = True
StopDist = 1.0
LinearSpeed = 0.5
ReturnFlag = True


def LoadConfigJson(file_path="TestConfig.json"):
    global OmegaMeasFlag, OmegaMeasDuration, OmegaValue
    global WantedYaw_deg, GotoWallFlag, StopDist, LinearSpeed
    global ReturnFlag
    if not os.path.exists(file_path):
        print(f"Fehler: Datei {file_path} nicht gefunden.")
        return

    with open(file_path, 'r') as file:
        try:
            # json.load liest die Datei direkt in ein Python-Dictionary
            data = json.load(file)
            # Zugriff mit .get() für Sicherheit gegen fehlende Keys
            OmegaMeasFlag     = data.get('OmegaMeasFlag', OmegaMeasFlag)
            OmegaMeasDuration = data.get('OmegaMeasDuration', OmegaMeasDuration)
            OmegaValue        = data.get('OmegaValue', OmegaValue)
            WantedYaw_deg     = data.get('WantedYaw_deg', WantedYaw_deg)
            GotoWallFlag      = data.get('GotoWallFlag', GotoWallFlag)
            StopDist          = data.get('StopDist', StopDist)
            LinearSpeed       = data.get('LinearSpeed', LinearSpeed)
            ReturnFlag        = data.get('ReturnFlag', ReturnFlag)
        except json.JSONDecodeError as exc:
            print(f"Fehler beim Parsen der JSON: {exc}")


def NormalizeAngle(angle_rad):
    return (angle_rad + math.pi) % math.tau - np.pi

class TestTask:
    def __init__(self):
        self.StateOmegaMeasurement = 0
        self.StateAlignTheta = 1
        self.StateGotoWall = 2
        self.StateIdle = 3
        
    def Init(self, node, params, retvals):
        LoadConfigJson()
        print("TestTask Parameter:")
        print(f"  {OmegaMeasFlag=}")
        print(f"  {OmegaMeasDuration=} sec")
        print(f"  {OmegaValue=} rad/s")
        print(f"  {WantedYaw_deg=}°")
        print(f"  {GotoWallFlag=}")  
        print(f"  {StopDist=} m")
        print(f"  {LinearSpeed=}m/s")
        print(f"  {ReturnFlag=}")
        
        self.node = node
        self.yaw = np.radians(WantedYaw_deg)
        if OmegaMeasFlag:
            self.node.get_logger().info("Starting TestTask - Omega Measurement")
            self.State = self.StateOmegaMeasurement
        else:
            self.node.get_logger().info(f"Starting TestTask. Wanted Yaw: {WantedYaw_deg}°. Stop Distance: {StopDist:.2f}m")
            self.node.SetWantedTheta(self.yaw)
            self.State = self.StateAlignTheta
        self.counter = 0
        self.measOmega = 0
        self.measSpeed = 0
        self.returnDone = False
        
    def MeasureLinearSpeed(self, dist):
        if self.counter <= 30:
            self.startTime = time.time()
            self.startDist = dist
            self.node.odom.SetStartPoint(dist, self.yaw)
            #print(self.startTime, dist, self.startDist, self.counter)
        elif self.counter % 10 == 0:
            ct = time.time()
            #print(ct, self.startTime, dist, self.startDist, self.counter)
            dt = ct - self.startTime
            ds = self.startDist - dist
            self.measSpeed = ds/dt
            text = f"WantedYaw={np.rad2deg(self.yaw):.1f} Grad  Yaw={np.rad2deg(self.node.theta):.1f} Grad  Zaunabstand={dist:.2f}m   Speed={self.measSpeed:.2f} m/s"
            self.node.RvizPrint(text)
        self.node.odom.UpdatePos(dist)
        self.counter += 1
        
    def MeasureOmega(self):
        yaw = self.node.theta
        if self.counter <= 10:
            self.sumYaw = 0
            self.lastYaw = yaw
            self.startTime = time.time()
            self.node.SetVelocities(OmegaValue, 0.0)
        elif self.counter % 10 == 0:
            ct = time.time()
            dt = ct - self.startTime
            self.sumYaw += NormalizeAngle(yaw - self.lastYaw)
            self.measOmega = self.sumYaw/dt
            self.lastYaw = yaw
            
            text = f"Yaw: {np.rad2deg(yaw):.1f} Grad  Omega: {self.measOmega:.2f} rad/s = {math.tau/self.measOmega:.3f} s/U"
            self.node.RvizPrint(text)
            if (ct - self.startTime) >= OmegaMeasDuration:
                return True
        self.counter += 1
        return False

    def Step(self, scan_msg):
        #setStartPoint = False
        if self.State == self.StateOmegaMeasurement:
            ready = self.MeasureOmega()
            if ready:
                self.node.SetWantedTheta(self.yaw)
                self.State = self.StateAlignTheta
        
        elif self.State == self.StateAlignTheta:
            if self.node.wantedThetaReached:
                if GotoWallFlag:
                    self.counter = 0
                    self.node.SetDirection(self.yaw, LinearSpeed)
                    self.State = self.StateGotoWall
                else:
                    self.State = self.StateIdle

        if self.State == self.StateGotoWall:
            ranges = np.array(scan_msg.ranges)
            LsAngleRange = 10       # Lidar angle range (in deg) for distance check
            start_deg = params.LidarMaxAngle - LsAngleRange // 2
            end_deg = start_deg + LsAngleRange
            dist = np.min(ranges[start_deg:end_deg]) # + params.LidarX
            dist = float(dist)
            self.MeasureLinearSpeed(dist)
            targetReached = dist < StopDist
                
            if targetReached: 
                self.node.ResetDirection()
                self.node.RvizPrint(f"Test completed. Omega: {self.measOmega:.3f} rad/s, Speed: {self.measSpeed:.2f} m/s, Zaunabstand={dist:.2f}m")
                if ReturnFlag and not self.returnDone:
                    self.returnDone = True
                    self.yaw = NormalizeAngle(self.yaw + math.pi)
                    self.node.SetWantedTheta(self.yaw)
                    self.State = self.StateAlignTheta
                else:
                    self.State = self.StateIdle

        if self.State == self.StateIdle:
            return TaskState.Ready, None
            
        return TaskState.Running, None
