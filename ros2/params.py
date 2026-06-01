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
    MARKER_LINES = 20       # frame_id, point_color, sx1, sy1, ex1, ey1, ... sxN, syN, exN, eyN, line_color1, ... line_colorN
    MARKER_POINTS = 21      # frame_id, point_color, x1, x2, ... xN
    MARKER_DELETEALL = 22   # None
    LIDAR_DATA = 30         # range1, range2, ... rangeN
    POSE = 31               # x_cm, y_cm, yaw_deg
    TEXT = 32               # text_bytes
    
    # UDP port für Visualisierung auf Laptop
    PORT = 5006


# angle range = -(LidarMaxAngle-1) ... LidarMaxAngle
LidarMaxAngle = 180  # 120
LidarRangeMin = 0.1
LidarRangeMax = 20.0
LidarFreq_Hz = 10

# Abstand des Lidar in X-Richtung vom Mittelpunkt der Achse
#LidarX = 0.4
LidarX = 0.8
LinearVelocity = 0.5

SimPause = False        # Simulator beim Start im Pause-Modus
SimShowRays = False     # Simulator zeigt Lidar-Strahlen beim Start


def ReadYawOffset():
    with open("YawOffset_deg.txt", "r") as file:
        yawOffset_rad = np.radians(float(file.read()))
    return yawOffset_rad

def WriteYawOffset(yawOffset_rad):
    with open("YawOffsetNew_deg.txt", "w") as file:
        file.write(str(np.degrees(yawOffset_rad)))

config = 5
if config==0:
    RobotInitX = 15.00      # Im Garten beim Gartentor
    RobotInitY =  9.00
    RobotInitTheta = np.pi
elif config==1:
    RobotInitX = -13.60     # unterhalb des Schuppen
    RobotInitY =   4.20
    RobotInitTheta = 0.0
elif config==2:
    RobotInitX = 12.00      # unterhalb der Terrasse
    RobotInitY = -3.00
    RobotInitTheta = 0.0
elif config==3:
    RobotInitX =  4.10      # links vor der Terrasse
    RobotInitY =  4.50
    RobotInitTheta = 0.0
elif config==4:
    RobotInitX =   -2.00    # rechts vom Schuppen 
    RobotInitY =   10.50
    RobotInitTheta = 0.0
elif config==5:         # für Mow Test
    RobotInitX = 18.0  # 19.0
    RobotInitY =  0.00
    RobotInitTheta = 0.0
elif config==6:
    RobotInitX = 19.00      # Im Wald beim Gartentor
    RobotInitY = 15.00
    RobotInitTheta =  -np.pi/2
elif config==7:
    RobotInitX =   -9.00    # vor der Schuppentür
    RobotInitY =    9.00
    RobotInitTheta = np.pi/2
else:
    RobotInitX = 0.0
    RobotInitY = 0.0
    RobotInitTheta = 0.0  # -np.pi
