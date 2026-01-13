#!/usr/bin/env python3
# split_and_merge.py
# Effiziente Liniensegmentierung für 2D-LiDAR-Scans
# Das Verfahren ist nur für Schuppen aussen geeignet!

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# --- Beispielpunktwolke mit Ecken ---
def OLDcreate_points():
    np.random.seed(0)
    # Rechteck-Kontur (4 Linien mit Rauschen)
    pts = []
    for x in np.linspace(0, 50, 60): pts.append((x, 0 + np.random.randn()*0.2))
    for y in np.linspace(0, 40, 50): pts.append((50 + np.random.randn()*0.2, y))
    for x in np.linspace(50, 0, 60): pts.append((x, 40 + np.random.randn()*0.2))
    for y in np.linspace(40, 0, 50): pts.append((0 + np.random.randn()*0.2, y))
    return np.array(pts)

def create_points():
    # Read Excel file
    num = 2
    if num==0: name = "wald_raw.xlsx"     
    if num==1: name = "garten-tor.xlsx"    
    if num==2: name = "schuppen-aussen.xlsx"    # Das Verfahren ist nur für Schuppen aussen geeignet
    if num==3: name = "schuppen-innen.xlsx"    

    df = pd.read_excel(f"data/{name}")    

    angles = df["Angle"]      # column named “Angle”
    radius = df["Range"]

    #----------------------------------
    # preprocessing(angles, radius)
    #----------------------------------
    # Winkel-Filter
    maxWinkel = 90  # 45
    mask = (angles >= 180-maxWinkel) & (angles <= 180+maxWinkel)
    angles = angles[mask]
    radius = radius[mask]

    mask = (radius > 0.1) & (radius < 7)
    angles = angles[mask]
    radius = radius[mask]

    angles = np.array(angles)
    radius = np.array(radius)

    angles = np.radians(270-angles)
    radius = np.array(radius)

    #points = [ ]
    #for i in range(len(angles)):
    #    x = radius[i] * np.cos(angles[i])
    #    y = radius[i] * np.sin(angles[i])
    #    points.append((x, y))
    #return np.array(points)
    
    x = radius * np.cos(angles)
    y = radius * np.sin(angles)
    return np.stack((x, y), axis=1)


def OLDsplit(points, threshold):
    if len(points) < 2:
        return []
    a, b = points[0], points[-1]
    # Abstand aller Punkte zur Verbindungslinie
    dists = np.array([point_line_dist(p, a, b) for p in points])
    i_max = np.argmax(dists)
    d_max = dists[i_max]

    # Split-Bedingung
    if d_max > threshold:
        left = split(points[:i_max+1], threshold)
        right = split(points[i_max:], threshold)
        return left + right
    else:
        return [(a, b)]

# --- Abstand Punkt -> Linie ---
def point_line_dist(p, a, b):
    # a,b: Endpunkte der Linie
    if np.allclose(a, b): return np.linalg.norm(p - a)
    return np.abs(np.cross(b - a, a - p)) / np.linalg.norm(b - a)

# --- Rekursiver Split ---
def split(points, threshold, min_points=100):
    if len(points) < min_points:
        return []

    a, b = points[0], points[-1]
    dists = np.array([point_line_dist(p, a, b) for p in points])
    i_max = np.argmax(dists)
    d_max = dists[i_max]

    if d_max > threshold and i_max not in (0, len(points)-1):
        left = split(points[:i_max+1], threshold, min_points)
        right = split(points[i_max:], threshold, min_points)
        return left + right
    else:
        # nur akzeptieren, wenn genügend Punkte
        return [(a, b)] if len(points) >= min_points else []

segments = None

def SegBasedDetection(points, threshold, min_points=100):
    global segments
    segments = split(points, threshold)

    numSegs = len(segments)
    print(f"{numSegs} Liniensegmente erkannt.")
    for seg in segments:
        print(seg)

    if numSegs > 1:
        pfosten1 = segments[0][1][0] + 1j*segments[0][1][1]
        pfosten2 = segments[1][0][0] + 1j*segments[1][0][1]
        tor = pfosten2 - pfosten1
        torBreite = abs(tor)
        print(f"{torBreite=}")
        torMitte = (pfosten2 + pfosten1) / 2
        startPoint = torMitte + 1 * tor / torBreite * (-1j)
        return torMitte, startPoint
    else:
        return None, None

# --- Merge benachbarter Segmente ---
def merge(segments, angle_thresh=np.deg2rad(5)):
    merged = []
    for seg in segments:
        if not merged:
            merged.append(seg)
            continue
        a1, b1 = merged[-1]
        a2, b2 = seg
        v1 = b1 - a1
        v2 = b2 - a2
        angle = np.arccos(np.clip(np.dot(v1, v2) /
                                  (np.linalg.norm(v1)*np.linalg.norm(v2)), -1, 1))
        if angle < angle_thresh:
            merged[-1] = (a1, b2)  # zusammenführen
        else:
            merged.append(seg)
    return merged

# --- Hauptprogramm ---
if __name__ == "__main__":
    points = create_points()
    threshold = 0.1      # maximaler Abstand zur Linie [m]
    
    torMitte, startPoint = SegBasedDetection(points, threshold)
    # --- Visualisierung ---
    plt.figure(figsize=(6,6))
    plt.scatter([torMitte.real, startPoint.real], [torMitte.imag, startPoint.imag], s=8, c='red')
    plt.scatter(points[:,0], points[:,1], s=8, c='gray')
    for (a,b) in segments:
        plt.plot([a[0], b[0]], [a[1], b[1]], 'r-', lw=2)
    plt.axis('equal')
    plt.show()
