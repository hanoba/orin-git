import numpy as np

# angle range = -120 ... 120 = -LidarMaxAngle ... LidarMaxAngle
LidarMaxAngle = 120
LidarFreq_Hz = 10

# Abstand des Lidar in X-Richtung vom Mittelpunkt der Achse
#LidarX = 0.4
LidarX = 0.0

config = 0
if config==1:
    BackWheelDrive = True
    RobotInitX = 15.00
    RobotInitY = 7.50
    RobotInitTheta =  -np.pi+3/4*np.pi
else:
    BackWheelDrive = False
    RobotInitX = 0.0
    RobotInitY = 0.0
    RobotInitTheta = 0.0  # -np.pi
