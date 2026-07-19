#!/usr/bin/env python3
import math
import numpy as np
from params import LidarMaxAngle, LidarRangeMax


# Winkel auf -pi bis pi normalisieren
def NormalizeAngle(angle_rad):
    return (angle_rad + math.pi) % math.tau - math.pi

# Hilfsfunktion: Sektor mit Wraparound in Grad
# Gibt Minimalradius in einen Bereich [start_deg, end_deg) zurück.
def GetSectorMin(ranges_np, start_deg, end_deg):
    start_deg += LidarMaxAngle
    end_deg += LidarMaxAngle
    if start_deg < end_deg: return np.nanmin(ranges_np[start_deg:end_deg]) 
    min1 = np.nanmin(ranges_np[start_deg:])
    min2 = np.nanmin(ranges_np[:end_deg])
    return min(min1, min2)  

# Hilfsfunktion: Sektor mit Wraparound in Grad
# Gibt den Median in einen Bereich [start_deg, end_deg) zurück.
# Kann so interpretiert werden, dass die Hälfte der Werte kleiner oder gleich dem Median sind.
# Das sollte helfen einzelne Grashalme zu ignorieren.
def GetSectorMedian(ranges_np, start_deg, end_deg):
    start_deg += LidarMaxAngle
    end_deg += LidarMaxAngle
    if start_deg < end_deg: return np.median(ranges_np[start_deg:end_deg]) 
    new_ranges = np.append(ranges_np[start_deg:], ranges_np[:end_deg])
    return np.median(new_ranges)

# Hilfsfunktion: Sektor mit Wraparound in Grad
# Berechnet den Minimalwert, der von N Range-Werten nicht überschritten wird.
# Das sollte helfen einzelne Grashalme zu ignorieren.
def GetRobustSectorMin(ranges_np, start_deg, end_deg, N):
    start_deg += LidarMaxAngle
    end_deg += LidarMaxAngle
    if start_deg < end_deg: 
        new_ranges = ranges_np[start_deg:end_deg]
    else:
        new_ranges = np.append(ranges_np[start_deg:], ranges_np[:end_deg])
        
    # Sicherheitscheck: Gibt es überhaupt N Werte?
    if len(new_ranges) < N:
        # Wenn nicht genug Daten da sind, gib den sicheren Maximalwert 
        return LidarRangeMax
    sorted_ranges = np.sort(new_ranges)
    return sorted_ranges[N-1]
    
