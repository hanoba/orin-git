#!/usr/bin/env python3
# PlotScans.py: Zeigt einen Datensatz an und führt eine Tor-Erkennung durch

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import cmath
import math


# Torbreite 
GATE_WIDTH_MIN = 0.92 - 0.15
GATE_WIDTH_MAX = 0.92 + 0.10

# Gate‑Detektion
GATE_MIN_ANGULAR_WIDTH_DEG = 140      # minimale zusammenhängende freie Winkelbreite
GATE_FREE_RANGE_THRESH = 6            # Schwelle in Meter ab der ein Strahl „frei“ gilt
GATE_START_DIST = 2



def detect_gate(angle, radius):
    """Erkennt die breiteste „freie“ Winkelmenge im 360°‑LiDAR.

    Vorgehen
      1) Klassifiziere jeden Strahl als frei (1) oder belegt (0) via Schwellwert.
      2) Finde die längste zusammenhängende Sequenz von 1en. Zirkulär behandeln,
         daher wird die Liste zu sich selbst konkateniert.
      3) Liefere den Mittelindex und dessen Winkel (in Rad) zurück.

    Rückgabe
      (mid_angle, length) oder None, falls kein genügend breites Gate gefunden wird.
    """
    n = len(radius)

    # Suche ersten Punkt des Zaunes
    i0 = None
    for i in range(n):
        if radius[i] < GATE_FREE_RANGE_THRESH: 
            i0 = i
            break
    if i0 == None: 
        print ("Zaun nicht gefunden")       #HB
        return None
    
    # Suche Tor
    cur_len = 0
    i2 = 0
    
    #print(f"{i0=} {n=}")
    gateFreeRangeThresh = GATE_FREE_RANGE_THRESH
    for i in range(i0+1,n):
        d = radius[i]
        if d >= gateFreeRangeThresh:
            if cur_len == 0:
                i1 = i-1
                gateFreeRangeThresh = radius[i1] + 0.5
            cur_len += 1
        else:
            if cur_len > 0: 
                i2 = i
                cur_len = 0
                gateFreeRangeThresh = GATE_FREE_RANGE_THRESH
        
                #print (f"{i1=}, {i2=}")
                # Erster Pfosten als complexe Zahl
                phi1 = angle[i1]
                d1 = radius[i1]
                pfosten1 = cmath.rect(d1, phi1)

                # Zweiter Pfosten als complexe Zahl
                phi2 = angle[i2]
                d2 = radius[i2]
                pfosten2 = cmath.rect(d2, phi2)

                #print(pfosten1, pfosten2)
                # die Breite des Tores muss zwischen GATE_WIDTH_MIN und GATE_WIDTH_MAX liegen
                tor = pfosten2 - pfosten1
                torBreite = abs(tor)
                print (f"{torBreite=}, {i1=}, {i2=}")
                #print(f"{torBreite=} {G(phi1)=} {d1=} {i1=}   {G(phi2)=} {d2=} {i2=}")      #HB
                if torBreite <= GATE_WIDTH_MAX and torBreite >= GATE_WIDTH_MIN:
                    break
        
    #if best_len == 0 or i2 == 0:
    #    print ("Tor nicht gefunden")       #HB
    #    return None, None
        
    if torBreite > GATE_WIDTH_MAX or torBreite < GATE_WIDTH_MIN:
        print (f"{torBreite=} passt nicht")       #HB
        return None, None
        
    # Winkel zum Mittelpunkt des Tores
    torMitte = (pfosten2 + pfosten1) / 2
    startPoint = torMitte + GATE_START_DIST * tor / torBreite * 1j
    print(f"{pfosten1=}, {pfosten2=}")
    return torMitte, startPoint



# Read Excel file
#S, name = 8,   "wald_raw.xlsx"       # reads the first sheet by default
#S, name = 5,   "garten-tor.xlsx"    
S, name = 5,   "schuppen-aussen.xlsx"    
#S, name = 2.5, "schuppen-innen.xlsx"    


df = pd.read_excel(f"data/{name}")    

#print(df.head())                      # show first rows

angles = df["Angle"]      # column named “Angle”
radius = df["Range"]

# Filter: nur Winkel 90°–270°
mask = (angles >= 90) & (angles <= 270)
angles = angles[mask]
radius = radius[mask]

mask = (radius > 0.1)
angles = angles[mask]
radius = radius[mask]

angles = np.array(angles)
radius = np.array(radius)

angles = np.radians(270-angles)
radius = np.array(radius)

angles = np.flip(angles)
radius = np.flip(radius)

print(f"{len(radius)=}, {len(angles)=}")
x = radius * np.cos(angles)
y = radius * np.sin(angles)

        
for i in range(15):
    print(i, angles[i]*360/math.tau, x[i])


torMitte, startPoint = detect_gate(angles, radius)
#print(torMitte, startPoint)

plt.figure()
plt.plot(x, y, '.')
if torMitte != None: 
    plt.plot([torMitte.real, startPoint.real], [torMitte.imag, startPoint.imag], 'o', color="red")
plt.xlim(-S, S)      # Bereich der X-Achse
plt.ylim(-1, S)      # Bereich der Y-Achse
plt.xlabel("X-Achse [m]")
plt.ylabel("Y-Achse [m]")
plt.title(name)
plt.grid(True)
#plt.axis('equal')
plt.show()
