import numpy as np
import params
import json
import os
from params import TaskState

# Task zum Testen des E-Karrens
# Der E-Karren wird zunächst auf WantedYaw_deg ausgerichtet
# Dann fährt er in diese Richtung bis er ein Hindernis erreicht.

# Default parameter
WantedYaw_deg = -90.0
GotoWallFlag = True
StopDist = 1.0
LinearSpeed = 0.5


def LoadConfigJson(file_path="TestConfig.json"):
    global WantedYaw_deg, GotoWallFlag, StopDist, LinearSpeed
    if not os.path.exists(file_path):
        print(f"Fehler: Datei {file_path} nicht gefunden.")
        return

    with open(file_path, 'r') as file:
        try:
            # json.load liest die Datei direkt in ein Python-Dictionary
            data = json.load(file)
            # Zugriff mit .get() für Sicherheit gegen fehlende Keys
            WantedYaw_deg   = data.get('WantedYaw_deg')
            GotoWallFlag    = data.get('GotoWallFlag')
            StopDist    = data.get('StopDist')
            LinearSpeed = data.get('LinearSpeed')
        except json.JSONDecodeError as exc:
            print(f"Fehler beim Parsen der JSON: {exc}")


class TestTask:
    def __init__(self):
        self.StateAlignTheta = 0
        self.StateGotoWall = 1
        self.StateIdle = 2
        
    def Init(self, node, params, retvals):
        LoadConfigJson()
        print(f"TestTask Parameter: {WantedYaw_deg=}°  {GotoWallFlag=}  {StopDist=}m  {LinearSpeed=}m/s")
        self.node = node
        self.yaw = np.radians(WantedYaw_deg)
        self.node.get_logger().info(f"Starting TestTask. Wanted Yaw: {WantedYaw_deg}°. Stop Distance: {StopDist:.2f}m")
        self.node.SetWantedTheta(self.yaw)
        self.State = self.StateAlignTheta
        self.counter = 0
        
    def ShowInfo(self, dist):
        if self.counter % 10 == 0:
            text = f"WantedYaw={np.rad2deg(self.yaw):.1f} Grad  Yaw={np.rad2deg(self.node.theta):.1f} Grad  Zaunabstand={dist:.2f}m"
            self.node.RvizPrint(text)
        self.counter += 1

    def Step(self, scan_msg):
        setStartPoint = False
        if self.State == self.StateAlignTheta:
            if self.node.wantedThetaReached:
                self.State = self.StateGotoWall
                if GotoWallFlag:
                    setStartPoint = True
                    self.node.SetDirection(self.yaw, LinearSpeed)
                else:
                    self.State = self.StateIdle

        if self.State == self.StateGotoWall:
            ranges = np.array(scan_msg.ranges)
            LsAngleRange = 10       # Lidar angle range (in deg) for distance check
            start_deg = params.LidarMaxAngle - LsAngleRange // 2
            end_deg = start_deg + LsAngleRange
            dist = np.min(ranges[start_deg:end_deg]) # + params.LidarX
            dist = float(dist)
            self.ShowInfo(dist)
            targetReached = dist < StopDist
            if setStartPoint: self.node.odom.SetStartPoint(dist, self.yaw)
            self.node.odom.UpdatePos(dist)
            if targetReached: 
                self.node.ResetDirection()
                self.State = self.StateIdle
                self.node.RvizPrint(f"Test completed. Zaunabstand={dist:.2f}m")
                self.State = self.StateIdle

        if self.State == self.StateIdle:
            return TaskState.Ready, None
            
        return TaskState.Running, None
