import numpy as np
import time
import math
import Ransac
import TaskLists
import params

from params import TaskState, Udp
from UdpSend import UdpSend, UdpPrint

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
    def __init__(self, pubVelocities):

        # Funktion zum setzen von Omega und vLinear
        self.PubVelocities = pubVelocities

        # Odometrie-Funktionen
        self.odom = Odometry()

        self.is_processing = False
        self.theta = 0.0
        self.wantedTheta = 0.0
        self.wantedThetaReached = True
        self.K_head = 0.3   #1.0
        self.angularMax = 2.0*2
        self.angular = 0.0
        self.linear = 0.0
        self.directionFlag = False
        self.simTimeSec = 0.0

        self.retvals = None

        # Set empty task list
        self.Reset()
        self.missedScans = 0

        # Set initial tasklist
        #taskList = TaskLists.Localization_TaskList
        taskList = TaskLists.Mowing_TaskList
        #taskList = TaskLists.Fahre_zum_Schuppen_TaskList
        #taskList = TaskLists.Fahre_in_den_Wald_TaskList
        #taskList = TaskLists.Fahre_in_den_Garten_TaskList
        #taskList = TaskLists.Fahre_hinters_Haus_TaskList
        #taskList = TaskLists.Bestimme_YawOffset_TaskList
        #taskList = TaskLists.Test_TaskList

        taskListName = taskList["name"]
        print(f"Trigger empfangen für {taskListName}")
        self.NewTaskList(taskList)

    def SetVelocities(self, omega, vLinear):
        self.angular = 0.0
        self.linear = 0.0
        self.wantedThetaReached = True
        self.directionFlag = False
        self.PubVelocities(vLinear, omega)

    def SetWantedTheta(self, wantedTheta):
        self.wantedTheta = wantedTheta
        self.wantedThetaReached = False
        self.directionFlag = False
        print(f"[{self.simTimeSec:.3f}] [SetWantedTheta] wantedTheta={self.wantedTheta}")
        self.linear = 0.0

    def SetDirection(self, theta, vLinear):
        self.directionFlag = True
        self.wantedTheta = theta
        self.linear = vLinear
        self.wantedThetaReached = True

    def ResetDirection(self):
        self.SetVelocities(0.0, 0.0)

    def CompassCallback(self, yaw):
        self.theta = NormalizeAngle(yaw)
        # self.theta += np.radians(10.0)    # for test of YawOffsetDetectionTask.py
        if self.directionFlag:
            e = self.wantedTheta - self.theta
            e = (e + math.pi) % math.tau - np.pi
            e = np.clip(e, -self.angularMax, self.angularMax)
            self.angular = e * self.K_head
            self.PubVelocities(self.linear, self.angular)
        elif not self.wantedThetaReached:
            e = self.wantedTheta - self.theta
            e = (e + math.pi) % math.tau - np.pi
            e = np.clip(e, -self.angularMax, self.angularMax)
            #print(f"{self.theta=}  {e=}  {self.wantedTheta=}")
            if abs(e) < np.deg2rad(10): # 0.002:
                self.wantedThetaReachedTime = time.time_ns() / 1e9
                self.wantedThetaReached = True
                print(f"[{self.wantedThetaReachedTime:.3f}] [CompassCallback] wantedThetaReached")
                self.angular = 0.0
            else:
                self.angular = e * self.K_head
            self.PubVelocities(self.linear, self.angular)

    def ScanCallback(self, ranges):
        if self.is_processing:
            self.missedScans += 1
            return
        start_zeit = time.perf_counter() # Zeitnahme startet
        self.is_processing = True
        # Lidardaten umdrehen bei FrontWheelDrive
        #if self.frontWheelDrive:
        #    scan_msg.ranges = scan_msg.ranges[::-1]
        #    angle_min = scan_msg.angle_min
        #    scan_msg.angle_min = -scan_msg.angle_max
        #    scan_msg.angle_max = -angle_min
        self.TaskStep(ranges)
        self.is_processing = False
        ende_zeit = time.perf_counter() # Zeitnahme endet
        dauer_ms = (ende_zeit - start_zeit) * 1000 # Umrechnung in Millisekunden

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
                round(theta_deg)      # Yaw in Grad
            ]
            UdpSend(udp_header, udp_data)

        # Ausgabe im Terminal (alle Sekunde, um das Terminal nicht zu fluten)
        #print(
        #    f"[ScanCallback] ⏱️ Rechenzeit: {dauer_ms:.2f} ms  "
        #    f"Missed Scans: {self.missedScans}  Theta={np.rad2deg(self.theta):.0f}°", throttle_duration_sec=10.0)

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
        UdpSend(Udp.MARKER_DELETEALL)

    def Reset(self):
        self.taskListName = "None"
        self.taskList = None
        self.taskIndex = 0
        self.SetVelocities(0.0, 0.0)
        print("Reset() called")

    def NewTaskList(self, taskList):
        self.taskListName = taskList["name"]
        self.taskList = taskList["tasks"]
        self.taskIndex = 0
        if self.taskIndex < len(self.taskList):
            task, params = self.taskList[self.taskIndex]
            task.Init(self, params, self.retvals)

    def TaskStep(self, ranges):
        if self.taskList is not None:
            if self.taskIndex >= len(self.taskList):
                self.Reset()
            else:
                task, params = self.taskList[self.taskIndex]
                status, self.retvals = task.Step(ranges)
                if status == TaskState.Ready:
                    nextTaskIndex = self.taskIndex+1
                    if nextTaskIndex < len(self.taskList):
                        self.GotoTask(nextTaskIndex)
                    else:
                        self.Reset()
                elif status == TaskState.Error:
                    print(f"ERROR [TaskStep] Error in task with task index: {self.taskIndex}")
                    self.Reset()

    def GotoTask(self, taskIndex):
        self.taskIndex = taskIndex
        if self.taskIndex < len(self.taskList):
            task, params = self.taskList[self.taskIndex]
            task.Init(self, params, self.retvals)
        else:
            print(f"ERROR [GotoTask] Illegal task index: {taskIndex}")

    def RvizPrint(self, text):
        mtext = f"{self.taskListName}: {text}"
        UdpPrint(mtext)
        print(mtext)
