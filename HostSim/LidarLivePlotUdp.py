#!/usr/bin/env python3
import numpy as np
#import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets
#import socket
#import select
import sys
from Lidar import LidarApp
from eKarrenLib import eKarren


bot = eKarren(debug=True)

def ProcessLidarData(angles, radius):
    vLinear = 1.0
    omega = 0.5
    bot.SetSpeed(vLinear, omega)
    return [0], [0]

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = LidarApp(ProcessLidarData)
    sys.exit(app.exec_())
    bot.Close()
    print("App terminated")
    