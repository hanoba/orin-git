import numpy as np

# angle range = -120 ... 120 = -LidarMaxAngle ... LidarMaxAngle
LidarMaxAngle = 120
LidarRangeMin = 0.1
LidarRangeMax = 20.0
LidarFreq_Hz = 10

# Abstand des Lidar in X-Richtung vom Mittelpunkt der Achse
#LidarX = 0.4
LidarX = 0.0

config = 3
if config==0:
    BackWheelDrive = True
    RobotInitX = 15.00
    RobotInitY = 7.50
    RobotInitTheta =  -np.pi+3/4*np.pi
elif config==1:
    BackWheelDrive = False
    RobotInitX = -13.60
    RobotInitY =   4.20
    RobotInitTheta = 0.0
elif config==2:
    BackWheelDrive = False
    RobotInitX = 12.00
    RobotInitY = -3.00
    RobotInitTheta = 0.0
elif config==3:
    BackWheelDrive = False
    RobotInitX =  4.10
    RobotInitY =  4.50
    RobotInitTheta = 0.0
else:
    BackWheelDrive = False
    RobotInitX = 0.0
    RobotInitY = 0.0
    RobotInitTheta = 0.0  # -np.pi
