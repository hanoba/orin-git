#!/usr/bin/env python3
# median.py: Führt eine Median-Filterung für "wald_raw.xlsx" durch

import pandas as pd
import numpy as np
from scipy.signal import medfilt
import matplotlib.pyplot as plt

def ReadData(name):
    df = pd.read_excel(f"data/{name}")    
    angles = df["Angle"]      # column named “Angle”
    radius = df["Range"]

    print(len(angles), len(radius))

    # Winkel-Filter
    maxWinkel = 170
    mask = (angles >= 180-maxWinkel) & (angles <= 180+maxWinkel)
    for i in range(0,2700,100):
        print(angles[i], mask[i])
    angles = angles[mask]
    radius = radius[mask]

    mask = (radius > 0.1)
    angles = angles[mask]
    radius = radius[mask]

    angles = np.array(270-angles)
    radius = np.array(radius)
    return angles, radius

angles, radius = ReadData("wald_raw.xlsx")

print(len(angles), len(radius))
# Simulierter LiDAR-Scan (Entfernung pro Winkel)
###angles = np.linspace(-180, 180, 720)
###radius = 3 + 0.2*np.random.randn(len(angles))
###radius[100] = 0.1   # Ausreißer
###radius[500] = 6.0   # weiterer Ausreißer

# Medianfilter (Fenstergröße 5)
filtered = medfilt(radius, kernel_size=5)

###plt.plot(angles, radius, label="Original", alpha=0.5)
###plt.plot(angles, filtered, label="Medianfilter (k=5)")
###plt.xlabel("Winkel [°]")
###plt.ylabel("Entfernung [m]")
###plt.legend()
###plt.show()

angles = np.radians(angles)
x = filtered * np.cos(angles)
y = filtered * np.sin(angles)


plt.figure()
plt.plot(x, y, '.')
#plt.xlim(-S, S)      # Bereich der X-Achse
#plt.ylim(-1, S)      # Bereich der Y-Achse
plt.xlabel("X-Achse [m]")
plt.ylabel("Y-Achse [m]")
#plt.title(name)
plt.grid(True)
#plt.axis('equal')
plt.show()