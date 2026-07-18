import numpy as np
import time
import math
import os
import Ransac
import TaskLists
import params

from params import TaskState, Udp
from UdpSend import UdpSend
from trace import Trace

def NormalizeAngle(angle_rad):
    return (angle_rad + math.pi) % math.tau - np.pi

def CheckAngle(angle_rad, value_rad):
    MaxDiff = np.deg2rad(5.0)
    diff1 = NormalizeAngle(angle_rad - value_rad)
    diff2 = NormalizeAngle(angle_rad + math.pi - value_rad)
    return abs(diff1) < MaxDiff or abs(diff2) < MaxDiff

def CheckAngleResult_deg(angle_rad, value_rad):
    diff1 = NormalizeAngle(angle_rad - value_rad)
    diff2 = NormalizeAngle(angle_rad + math.pi - value_rad)
    return np.rad2deg(min(abs(diff1), abs(diff2)))

def CheckLength(vektorLen, value):
    Margin = 0.50
    if vektorLen < value - Margin: return False
    if vektorLen > value + Margin: return False
    return True

def LineEquation(A, B):
    """ Berechne die Parameter a und b der Geraden y=a*x+b, die durch die Punkte A und B geht """
    a = (B[1] - A[1]) / (B[0] - A[0])
    b = A[1] - a*A[0]
    return a, b

def DistY(start, end):
    """ Berechne den Abstand in Y-Richtung von der Geraden, die durch die Punkte start und end geht """
    a, b = LineEquation(start, end)
    return b

# Odometrie-Funktionen zur Schätzung der Position
class Odometry:
    def __init__(self):
        self.pos = np.array([0.0, 0.0])
        self.theta = 0

    def SetPos(self, x, theta):
        """ Wird immer aufgerufen, wenn Position bestimmt wurde """
        self.pos = x
        self.theta = theta

    def GetPos(self):
        return float(self.pos[0]), float(self.pos[1])

    def SetStartPoint(self, dist, theta):
        """ Wird von FollowPathTask aufgerufen, wenn neues Pfadsegment Format 1 gestartet wird. """
        self.theta = theta
        self.startDist = dist
        self.startPos = self.pos
        self.heading = np.array([math.cos(theta), math.sin(theta)])

    def UpdatePos(self, dist):
        d = self.startDist - dist
        self.pos = self.startPos + d*self.heading



class Navigator:
    def __init__(self):
        # Odometrie-Funktionen
        self.odom = Odometry()
        
        # non-blocking Trace
        self.trace = Trace()

        self.is_processing = False
        self.theta = 0.0
        self.wantedTheta = 0.0
        self.wantedThetaReached = True
        self.K_head = 0.3*2   #1.0
        self.K_headFast = 10.0   
        self.angularMax = 2.0
        self.angular = 0.0
        self.linear = 0.0
        self.mode = 0
        self.directionFlag = False
        self.simTimeSec = 0.0

        self.retvals = None

        # Set empty task list
        self.Reset()
        self.missedScans = 0
        self.udp = UdpSend(
            Udp.PORT_VIZ, 
            os.environ.get('EKARREN_VIZ_IP1'),
            os.environ.get('EKARREN_VIZ_IP2'),
        )
        self.scanCallbackCounter = 0

    def SetMode(mode):
        self.mode = mode
    
    def SetVelocities(self, omega, vLinear):
        self.angular = omega    #0.0
        self.linear = vLinear   #0.0
        self.wantedThetaReached = True
        self.directionFlag = False
        #self.PubVelocities(vLinear, omega)

    def SetLinearVelocity(self, vLinear):
        self.linear = vLinear  

    def SetWantedTheta(self, wantedTheta, vLinear=0.0, turnRight=False, turnLeft=False):
        assert turnRight==False or turnLeft==False
        self.turnRight = turnRight
        self.turnLeft = turnLeft
        self.wantedTheta = NormalizeAngle(wantedTheta)
        self.wantedThetaReached = False
        self.directionFlag = False
        print(f"[{self.simTimeSec:.3f}] [SetWantedTheta] wantedTheta={np.degrees(self.wantedTheta)}°   theta={np.degrees(self.theta)}°")
        self.linear = vLinear
        if wantedTheta <= self.theta:
            self.wantedThetaSign = 1.0
            self.angularMin = -0.1
        else:
            self.wantedThetaSign = -1.0
            self.angularMin = 0.1

    def SetDirection(self, theta, vLinear):
        self.directionFlag = True
        self.wantedTheta = theta
        self.linear = vLinear
        self.wantedThetaReached = True
        print(f"[{self.simTimeSec:.3f}] [SetDirection] wantedTheta ={math.degrees(theta)}°")

    def ResetDirection(self):
        self.SetVelocities(0.0, 0.0)

    def CompassCallback(self, yaw):
        self.trace.Put("[CompassCallback] START")
        start_zeit = time.perf_counter_ns() # Zeitnahme startet
        self.theta = NormalizeAngle(yaw)
        # self.theta += np.radians(10.0)    # for test of YawOffsetDetectionTask.py
        if self.directionFlag:
            e = self.wantedTheta - self.theta
            e = (e + math.pi) % math.tau - math.pi
            self.angular = e * 4.0   #self.K_head
        elif not self.wantedThetaReached:
            e1 = self.wantedTheta - self.theta
            e = (e1 + math.pi) % math.tau - math.pi
            # Handling of fixed turning direction (used for 180° u-turns)
            ang = 0.0
            if abs(e) > np.pi/4:
                if self.turnRight: ang = -self.angularMax
                elif self.turnLeft: ang = self.angularMax
            else:
                self.turnRight = False
                self.turnLeft = False
            # Normal case
            if ang != 0.0: self.angular = ang
            elif e*self.wantedThetaSign > 0.0:
                self.wantedThetaReachedTime = time.time_ns() / 1e9
                self.wantedThetaReached = True
                print(f"[{self.wantedThetaReachedTime:.3f}] [CompassCallback] wantedThetaReached"
                        f" {self.wantedTheta=}  {self.theta=}  {self.wantedThetaSign}  {e=}  {e1=}")
                self.angular = 0.0
            else:
                #print(f"{self.theta=}  {e=}  {self.wantedTheta=}  {self.wantedThetaSign=}")
                self.angular = np.clip(e*self.K_headFast + self.angularMin,  -self.angularMax, self.angularMax)
        dauer_ms = (time.perf_counter_ns() - start_zeit) * 1e-6 # Umrechnung in ms
        self.trace.Put(f"[CompassCallback] END {dauer_ms=:.3f}")
        return self.linear, self.angular, self.mode

    def ScanCallback(self, ranges):
        self.trace.Put("[ScanCallback] START")
        start_zeit = time.perf_counter_ns() # Zeitnahme startet
        if self.is_processing:
            self.missedScans += 1
            self.trace.Put(f"[ScanCallback] {self.missedScans=}")
            return
        self.scanCallbackCounter += 1
        self.is_processing = True
        # Lidardaten umdrehen bei FrontWheelDrive
        #if self.frontWheelDrive:
        #    scan_msg.ranges = scan_msg.ranges[::-1]
        #    angle_min = scan_msg.angle_min
        #    scan_msg.angle_min = -scan_msg.angle_max
        #    scan_msg.angle_max = -angle_min
        self.TaskStep(ranges)
        self.is_processing = False

        # Publish estimated position
        if params.PublishEstimatedPosition:
            # Die geschätzte Position (Absolute Koordinaten auf der Karte)
            posX, posY = self.odom.GetPos()

            # POSE an Visualizer senden
            cm = 100.0
            theta_deg = np.degrees(self.theta)
            udp_header = Udp.POSE
            udp_data = [
                # round(x) gibt in Python 3 automatisch einen Integer zurück
                round(posX*cm),       # X-Koordinate in cm
                round(posY*cm),       # Y-Koordinate in cm
                round(theta_deg*10.0) # Yaw in 0.1 Grad-Einheiten
            ]
            self.udp.Send(udp_header, udp_data)

        # Ausgabe im Terminal (alle Sekunde, um das Terminal nicht zu fluten)
        #if self.scanCallbackCounter % 10 == 0: 
        #    print(f"[ScanCallback] #{self.scanCallbackCounter} ⏱️ Rechenzeit: {dauer_ms:.3f} ms  "
        #    f"Missed Scans: {self.missedScans}  Theta={np.rad2deg(self.theta):.0f}°")
        dauer_ms = (time.perf_counter_ns() - start_zeit) * 1e-6 # Umrechnung in ms
        self.trace.Put(f"[ScanCallback] END {dauer_ms=:.3f}")


    def Walldetector(self, ranges, angMin=-np.pi, angMax=np.pi):
        # ranges = np.array(msg.ranges)
        # #rangeMask = np.isfinite(ranges) & (ranges > params.LidarRangeMin + 0.01) & (ranges < params.LidarRangeMax - 0.1)
        # rangeMask = np.isfinite(ranges) & (ranges > msg.range_min + 0.01) & (ranges < msg.range_max - 0.1)
        # #valid = np.isfinite(ranges) & (ranges > 0.1) & (ranges < 30.0)
        # angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        angles = params.LidarAngles
        rangeMask = np.isfinite(ranges) & (ranges > params.LidarRangeMin + 0.01) & (ranges < params.LidarRangeMax - 0.1)
        angMask = (angles >=angMin) & (angles <= angMax)
        valid = rangeMask & angMask
        points = np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))[valid]
        all_detected_walls = Ransac.LineDetection(points)
        return all_detected_walls

    def Stop(self):
        self.Reset()
        self.RvizPrint("Reset")
        self.DeleteAllMarkers()

    def DeleteAllMarkers(self):
        # Alle Markers von viz.py löschen
        self.udp.Send(Udp.MARKER_DELETEALL)

    def Reset(self):
        self.taskListName = "None"
        self.taskListTasks = None
        self.taskIndex = 0
        self.SetVelocities(0.0, 0.0)
        print("Reset() called")

    def NewTaskList(self, taskList):
        self.taskListName = taskList["name"]
        tasks = taskList["tasks"]
        self.taskIndex = 0
        if self.taskIndex < len(tasks):
            task, params = tasks[self.taskIndex]
            task.Init(self, params, self.retvals)
        print(f"TaskList {self.taskListName} wird gestartet")
        # Die folgende Zuweisung darf erst erfolgen, nachdem task.Init() ausgeführt wurde!
        self.taskListTasks = tasks

    def TaskStep(self, ranges):
        if self.taskListTasks is not None:
            if self.taskIndex >= len(self.taskListTasks):
                self.Reset()
            else:
                task, params = self.taskListTasks[self.taskIndex]
                status, self.retvals = task.Step(ranges)
                if status == TaskState.Ready:
                    nextTaskIndex = self.taskIndex+1
                    if nextTaskIndex < len(self.taskListTasks):
                        self.GotoTask(nextTaskIndex)
                    else:
                        self.Reset()
                elif status == TaskState.Error:
                    print(f"ERROR [TaskStep] Error in task with task index: {self.taskIndex}")
                    self.Reset()

    def GotoTask(self, taskIndex):
        self.taskIndex = taskIndex
        if self.taskIndex < len(self.taskListTasks):
            task, params = self.taskListTasks[self.taskIndex]
            task.Init(self, params, self.retvals)
        else:
            print(f"ERROR [GotoTask] Illegal task index: {taskIndex}")

    def RvizPrint(self, text):
        mtext = f"{self.taskListName}: {text}"
        self.udp.Print(mtext)
        print(mtext)

    def PublishMarkers(self, all_detected_walls, isDetectedWallValid):
        num_lines = len(all_detected_walls)
        
        # UDP Kommando zum Zeichen von Linien
        udp_header = Udp.MARKER_LINES
        udp_data = [
            Udp.FRAME_LIDAR,    # Frame (FRAME_LIDAR oder FRAME_MAP)
            Udp.BLUE]           # Für Linien-Endpunkte blaue Kreise zeichnen (NONE = keine Kreise)
            # Es folgen die Linien sx, sy, ex, ey ...
            # und danach die Farben der Linien
        cm = 100.0  # zur Umrechnung von Meter in cm
        udp_line_colors = []
        
        for i, (start, end) in enumerate(all_detected_walls):
            if isDetectedWallValid[i]:
                udp_line_colors.append(Udp.RED)
            else: 
                udp_line_colors.append(Udp.GREEN)

            sx, sy, ex, ey = int(start[0]*cm), int(start[1]*cm), int(end[0]*cm), int(end[1]*cm)
            udp_data.extend([sx, sy, ex, ey])

        if num_lines > 0:
            udp_data.extend(udp_line_colors)
            self.udp.Send(udp_header, udp_data)
