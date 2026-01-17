#!/usr/bin/env python3

# RansacGateDetection.py  
# Verwendet RANSAC-basierte Torerkennung für die gemessenen Datensätze durch
# Der Algorithmus von ../HostSim/CornerDetectorNode.py (isaac-sim) wird verwendet.

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import cmath
import math
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets
import time

# Reuse von ../HostSim/CornerDetectorNode.py
class SmartCornerDetector():
    def __init__(self):
        self.dist_thresh = 0.05
        self.min_points = 6
        self.numIter = 40

        # Wenn Punkte weiter als max_gap auseinander liegen -> Tor/Lücke
        self.max_gap = 0.60 #0.50

    def find_intersection(self, line1, line2):
        p1, p2 = line1; p3, p4 = line2
        x1, y1 = p1; x2, y2 = p2; x3, y3 = p3; x4, y4 = p4
        denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
        if denom == 0: return None 
        ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom
        return np.array([x1 + ua * (x2 - x1), y1 + ua * (y2 - y1)])

    def get_lines_with_gap_check(self, points):
        """Findet die beste Linie und teilt sie bei Lücken auf."""
        
        if len(points) < self.min_points: return [], None
        
        pts = points.astype(np.float32)
        best_mask = None
        best_count = 0
        
        # 1. RANSAC 
        for _ in range(self.numIter):
            idx = np.random.choice(len(pts), 2, replace=False)
            p1, p2 = pts[idx[0]], pts[idx[1]]
            vec = p2 - p1
            norm = np.linalg.norm(vec)
            if norm < 0.01: continue 
            normal = np.array([-vec[1], vec[0]]) / norm
            dists = np.abs(np.dot(pts - p1, normal))
            
            mask = dists < self.dist_thresh
            count = np.sum(mask)
            if count > best_count:
                best_count = count
                best_mask = mask

        if best_count < self.min_points: return [], None

        # 2. Inlier verfeinern
        inliers = points[best_mask]
        mean = np.mean(inliers, axis=0)
        uu, dd, vv = np.linalg.svd(inliers - mean)
        direction = vv[0]
        
        # 3. GAP-CHECK (Tor-Erkennung)
        projections = np.dot(inliers - mean, direction)
        sort_idx = np.argsort(projections)
        sorted_inliers = inliers[sort_idx]
        sorted_proj = projections[sort_idx] # Wird hier nicht direkt genutzt, aber gut für debugging

        # Abstände zwischen aufeinanderfolgenden Punkten
        diffs = np.linalg.norm(sorted_inliers[1:] - sorted_inliers[:-1], axis=1)
        
        # KORREKTUR: Hier wurde max_gap schon korrekt genutzt (self.max_gap)
        gap_indices = np.where(diffs > self.max_gap)[0]

        split_lines = []
        last_idx = 0
        
        # Ein Segment sollte fast so viele Punkte haben wie min_points
        # Man kann hier etwas toleranter sein (z.B. min_points - 1) oder strikt bleiben.
        min_seg_len = max(3, self.min_points - 1) 

        # Wand an den Lücken zerteilen
        for gap_idx in gap_indices:
            segment = sorted_inliers[last_idx : gap_idx + 1]
            
            # KORREKTUR: Dynamische Länge statt harter 5
            if len(segment) >= min_seg_len: 
                p_start = segment[0]
                p_end = segment[-1]
                split_lines.append((p_start, p_end))
            last_idx = gap_idx + 1
        
        # Das letzte (oder einzige) Stück hinzufügen
        final_segment = sorted_inliers[last_idx:]
        
        # KORREKTUR: Dynamische Länge statt harter 5
        if len(final_segment) >= min_seg_len:
            split_lines.append((final_segment[0], final_segment[-1]))

        return split_lines, best_mask

    def run(self, angles, ranges):
        start_zeit = time.perf_counter() # Zeitnahme startet
        valid = np.isfinite(ranges) & (ranges > 0.1) & (ranges < 30.0)
        points = np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))[valid]

        all_detected_walls = []
        temp_points = points.copy()

        # Iterativ Linien suchen
        for _ in range(10):
            # Abbruchbedingung auch hier an min_points koppeln
            if len(temp_points) < self.min_points: break

            lines, mask = self.get_lines_with_gap_check(temp_points)
            if not lines: break
            
            all_detected_walls.extend(lines)
            temp_points = temp_points[~mask]

        ende_zeit = time.perf_counter() # Zeitnahme endet
        dauer_ms = (ende_zeit - start_zeit) * 1000 # Umrechnung in Millisekunden
        
        # Ausgabe im Terminal (alle Sekunde, um das Terminal nicht zu fluten)
        print(f"⏱️ Rechenzeit: {dauer_ms:.2f} ms")
        return all_detected_walls

def Preprocessing(angles, radius):
    # Winkel-Filter
    maxWinkel = 90
    mask = (angles >= 180-maxWinkel) & (angles <= 180+maxWinkel)
    angles = angles[mask]
    radius = radius[mask]

    mask = (radius > 0.1)
    angles = angles[mask]
    radius = radius[mask]

    angles = np.radians(270-angles)
    radius = np.array(radius)

    if True:
        angles = np.flip(angles)
        radius = np.flip(radius)

    x = radius * np.cos(angles)
    y = radius * np.sin(angles)
    points = np.stack((x, y), axis=1)

    return angles, radius, points        
    

def main():
    num = 1
    if num==0: name, xMin, xMax, yMin, yMax, distMin = "wald_raw.xlsx"        ,  -6,  8,  0, 10, 2.0  
    if num==1: name, xMin, xMax, yMin, yMax, distMin = "garten-tor.xlsx"      ,  -4, 10,  -4,  8, 2.0
    if num==2: name, xMin, xMax, yMin, yMax, distMin = "schuppen-aussen.xlsx" ,  -6,  8,  0,  6, 1.0
    if num==3: name, xMin, xMax, yMin, yMax, distMin = "schuppen-innen.xlsx"  ,  -4,  4,  0,  3, 0.5

    # --- Plot-Setup --------------------------------------------------------------
    app = QtWidgets.QApplication([])
    pg.setConfigOptions(antialias=True)
    win = pg.plot(title=name)
    win.setAspectLocked(True)
    win.setXRange(xMin, xMax)
    win.setYRange(yMin, yMax)
    win.showGrid(x=True, y=True)
    
    lidarData = win.plot(pen=None, symbol='o', symbolSize=2, symbolBrush=(0, 255, 0))
    lineEndPoints = win.plot(pen=None, symbol='o', symbolSize=8, symbolBrush=(255, 0, 0))

    detector = SmartCornerDetector()
    
    text = pg.TextItem(anchor=(0,1))
    win.getPlotItem().addItem(text)
    text.setPos(xMin/2, yMax-1)
    
    try:
        # Read Excel file

        df = pd.read_excel(f"data/{name}")    

        print(name, df.head())                      # show first rows

        angles = df["Angle"]      # column named “Angle”
        radius = df["Range"]
        
        angles = angles.to_numpy()
        radius = radius.to_numpy()

        angles, radius, points = Preprocessing(angles, radius)
        
        print(points.shape)

        all_detected_walls = detector.run(angles, radius)

        # Mittelpunkt (arithmetisches Mittel)
        center = points.mean(axis=0)

        # Euklidische Abstände der Punkte vom Mittelpunkt
        distances = np.linalg.norm(points - center, axis=1)

        # Standardabweichung 
        std_distance  = distances.std()

        line_pool = []
        
        k = 0
        xVals = []
        yVals = []
        for i, (start, end) in enumerate(all_detected_walls):
            dist = np.linalg.norm(start - end)
            if dist > distMin:
                print(f"Wall{k}: {start}   {end}   {dist}")
                new_line = win.plot(pen=pg.mkPen('y'))
                line_pool.append(new_line)
                x_coords = [float(start[0]), float(end[0])]
                y_coords = [float(start[1]), float(end[1])]
                line_pool[k].setData(x_coords, y_coords)
                k += 1
                xVals.append(start[0])
                xVals.append(end[0])
                yVals.append(start[1])
                yVals.append(end[1])
        lineEndPoints.setData(xVals, yVals)

        count = np.sum(radius < 1.5)
        print(f"----{name}:  {std_distance =} m   {count=}")
        
        lidarData.setData(points)

        n = name + "&nbsp;" * 10
        h=f"<div style=\"background-color:black; color:red; font-size:20pt;\">{n}</div>"
        text.setHtml(h)

        QtWidgets.QApplication.processEvents()    
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n⏹ Ende.")


if __name__ == '__main__':
    main()