import numpy as np

# angle range = -120 ... 120 = -LidarMaxAngle ... LidarMaxAngle
LidarMaxAngle = 120
LidarFreq_Hz = 10

config = 1
if config==1:
    RobotInitX = 15.00
    RobotInitY = 7.50
    RobotInitTheta =  -np.pi+3/4*np.pi
else:
    RobotInitX = 0.0
    RobotInitY = 0.0
    RobotInitTheta =  -np.pi
