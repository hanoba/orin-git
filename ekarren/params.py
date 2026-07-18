import math
import numpy as np
from enum import IntEnum

class Ort(IntEnum):
    ImGarten = 0
    ImWald = 1
    ImSchuppen = 2
    ImNirwana = 3

class TaskState(IntEnum):
    Running = 0
    Ready = 1
    Error = 2

class Udp(IntEnum):
    # Farben
    NONE = 0
    WHITE = 1
    RED = 2
    GREEN = 3
    BLUE = 4 
    
    # Frames
    FRAME_LIDAR = 10
    FRAME_MAP = 11
    
    # UDP-Headers           # Parameters (short) - All x/y coordinates in cm
    #-----------------------#------------------------------------------------------------------------------------------------
    MARKER_LINES = 20       # frame_id, point_color, sx1, sy1, ex1, ey1, ... sxN, syN, exN, eyN, line_color1, ... line_colorN
    MARKER_POINTS = 21      # frame_id, point_color, x1, x2, ... xN
    MARKER_DELETEALL = 22   # None
    LIDAR_DATA = 30         # range1, range2, ... rangeN
    POSE = 31               # x_cm, y_cm, yaw_deg
    TEXT = 32               # text_bytes
    TELEOP = 40             # vLinear*1000, vAngular*1000
    
    # UDP port für Visualisierung auf Laptop
    PORT_VIZ = 5006
    PORT_TELEOP = 5007

Ardumower = True

# angle range = -(LidarMaxAngle-1) ... LidarMaxAngle
LidarMaxAngle = 180 
LidarRangeMin = 0.01    # from YDLIDAR (old 0.1)
LidarRangeMax = 64.0    # from YDLIDAR (old 20.0)
LidarFreq_Hz = 10
LidarAngleMin = math.radians(1-LidarMaxAngle)
LidarAngleMax = math.radians(LidarMaxAngle)
LidarAngleIncrement = (LidarAngleMax - LidarAngleMin) / (2*LidarMaxAngle - 1)
LidarAngles = LidarAngleMin + np.arange(2*LidarMaxAngle) * LidarAngleIncrement


# Abstand des Lidar in X-Richtung vom Mittelpunkt der Achse
#LidarX = 0.4
LidarX = -0.25 if Ardumower else 0.8
LinearVelocity = 0.5

SimPause = False        # Simulator beim Start im Pause-Modus
SimShowRays = False     # Simulator zeigt Lidar-Strahlen beim Start

PublishEstimatedPosition = True

def ReadYawOffset():
    with open("YawOffset_deg.txt", "r") as file:
        yawOffset_rad = np.radians(float(file.read()))
    return yawOffset_rad

def WriteYawOffset(yawOffset_rad):
    with open("YawOffsetNew_deg.txt", "w") as file:
        file.write(str(np.degrees(yawOffset_rad)))

# config = 0
# if config==0:   pose = (15.00, 9.00, np.pi)         # Im Garten beim Gartentor
# elif config==1: pose = (-13.60, 4.20, 0.0)          # unterhalb des Schuppen
# elif config==2: pose = (12.00, -3.00, 0.0)          # unterhalb der Terrasse
# elif config==3: pose = (4.10, 4.50, 0.0)            # links vor der Terrasse
# elif config==4: pose = (-2.00, 10.50, 0.0)          # rechts vom Schuppen 
# elif config==5: pose = (18.0, 0.00, 0.0)            # für Mow Test
# elif config==6: pose = (19.00,  15.00, -np.pi/2)    # Im Wald beim Gartentor
# elif config==7: pose = (-9.00, 9.00, np.pi/2)       # vor der Schuppentür
# else:           pose = (0.00, 0.00, 0.0)
# (RobotInitX, RobotInitY, RobotInitTheta) = pose
