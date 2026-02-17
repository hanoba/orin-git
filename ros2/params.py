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

# angle range = -120 ... 120 = -LidarMaxAngle ... LidarMaxAngle
LidarMaxAngle = 160
LidarRangeMin = 0.1
LidarRangeMax = 20.0
LidarFreq_Hz = 10

# Abstand des Lidar in X-Richtung vom Mittelpunkt der Achse
#LidarX = 0.4
LidarX = 0.0
LinearVelocity = 0.5

SimPause = False
SimShowRays = False

config = 0
if config==0:
    BackWheelDrive = False
    RobotInitX = 15.00      # Im Garten beim Gartentor
    RobotInitY = 7.50
    RobotInitTheta =  -np.pi+3/4*np.pi
elif config==1:
    BackWheelDrive = False
    RobotInitX = -13.60     # unterhalb des Schuppen
    RobotInitY =   4.20
    RobotInitTheta = 0.0
elif config==2:
    BackWheelDrive = False
    RobotInitX = 12.00      # unterhalb der Terrasse
    RobotInitY = -3.00
    RobotInitTheta = 0.0
elif config==3:
    BackWheelDrive = False
    RobotInitX =  4.10      # links vor der Terrasse
    RobotInitY =  4.50
    RobotInitTheta = 0.0
elif config==4:
    BackWheelDrive = False
    RobotInitX =   -2.00    # rechts vom Schuppen 
    RobotInitY =   10.50
    RobotInitTheta = 0.0
elif config==5:         # für Mow Test
    BackWheelDrive = False
    RobotInitX = 18.0  # 19.0
    RobotInitY =  0.00
    RobotInitTheta = 0.0
elif config==6:
    BackWheelDrive = False
    RobotInitX = 19.00      # Im Wald beim Gartentor
    RobotInitY = 15.00
    RobotInitTheta =  -np.pi/2
else:
    BackWheelDrive = False
    RobotInitX = 0.0
    RobotInitY = 0.0
    RobotInitTheta = 0.0  # -np.pi
