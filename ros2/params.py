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

# angle range = -(LidarMaxAngle-1) ... LidarMaxAngle
LidarMaxAngle = 180
LidarRangeMin = 0.1
LidarRangeMax = 20.0
LidarFreq_Hz = 10

# Abstand des Lidar in X-Richtung vom Mittelpunkt der Achse
#LidarX = 0.4
LidarX = 0.8
LinearVelocity = 0.5

SimPause = True        # Simulator beim Start im Pause-Modus
SimShowRays = True      # Simulator zeigt Lidar-Strahlen beim Start

config = 7
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
