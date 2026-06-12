import numpy as np
import Ransac
from params import TaskState, Udp
from params import ReadYawOffset, WriteYawOffset
import params
from UdpSend import UdpSend

class YawOffsetDetectionTask:
    def Init(self, node, params, retvals=None):
        self.node = node
        self.node.SetWantedTheta(np.radians(90.0))
        self.numMeas = 0
        self.maxMeas = 10
        self.sumAngle_rad = 0.0
        self.yawOffsetOld = ReadYawOffset()

    def Step(self, ranges):
        if not self.node.wantedThetaReached:
            return TaskState.Running, None

        all_detected_walls = self.Walldetector(ranges) 
        walls = np.array(all_detected_walls)

        linie, laenge, w_rad = self.ComputeLongestLine(walls)
        self.sumAngle_rad += w_rad
        
        self.numMeas += 1    
        w_grad = np.degrees(w_rad)
        
        yawOld = self.node.theta
        yawNew = -w_rad
        yawOffsetNew = yawNew - yawOld + self.yawOffsetOld
        
        print(f"--- Messung {self.numMeas} ---")
        print(f"Punkte der längsten Linie: {linie[0,:]}, {linie[1,:]}")
        print(f"Länge:                     {laenge:.2f}")
        print(f"Winkel (Bogenmaß):         {w_rad:.4f} rad")
        print(f"Winkel (Grad):             {w_grad:.2f}°")
        print(f"YawOffsetNew (Grad):       {np.degrees(yawOffsetNew):.2f}°")

        self.PublishMarkers(walls, linie)
        if self.numMeas < self.maxMeas:
            return TaskState.Running, None
          
        w_rad = self.sumAngle_rad / self.maxMeas
        yawNew = -w_rad
        yawOffsetNew = yawNew - yawOld + self.yawOffsetOld
        WriteYawOffset(yawOffsetNew)
        print("--- Endergebnis ---")
        print(f"YawOffsetOld (Grad):       {np.degrees(self.yawOffsetOld):.2f}°")
        print(f"YawOffsetNewMean (Grad):   {np.degrees(yawOffsetNew):.2f}°")
        return TaskState.Ready, None
    
    def Walldetector(self, ranges):
        #ranges = np.array(msg.ranges)
        #angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        angles = params.LidarAngles
        anglesMask = (angles > -np.pi/2) & (angles < np.pi/2)
        rangesMask = np.isfinite(ranges) & (ranges > params.LidarRangeMin + 0.01) & (ranges < params.LidarRangeMax - 0.1)
        points = np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))[anglesMask & rangesMask]
        allDetectedWalls = Ransac.LineDetection(points)
        return allDetectedWalls
        
    def ComputeLongestLine(self, walls):
        # 1. Berechne die Differenzvektoren (Endpunkt - Anfangspunkt) für alle Linien
        # walls[:, 1, :] sind alle Endpunkte, walls[:, 0, :] sind alle Anfangspunkte
        vektoren = walls[:, 1, :] - walls[:, 0, :]
        
        # 2. Berechne die Längen aller Vektoren
        # np.linalg.norm mit axis=1 berechnet die euklidische Länge für jede Zeile
        laengen = np.linalg.norm(vektoren, axis=1)
        
        # 3. Finde den Index der längsten Linie
        max_index = np.argmax(laengen)
        
        # 4. Extrahiere die längste Linie und ihren Vektor
        #    Vektor muss nach rechts zeigen
        laengste_linie = walls[max_index]
        if laengste_linie[1][1] < laengste_linie[0][1]:
            laengster_vektor = vektoren[max_index]
        else:
            laengster_vektor = -vektoren[max_index]
        
        # 5. Berechne den Winkel mit arctan2 (gibt den Winkel im Bogenmaß zurück)
        # arctan2(y, x) beachtet den Quadranten korrekt
        winkel_rad = np.arctan2(laengster_vektor[1], laengster_vektor[0])
        
        return laengste_linie, laengen[max_index], winkel_rad


    def PublishMarkers(self, walls, punkte):
        num_lines = len(walls)
    
        # UDP Kommando zum Zeichen von Linien
        udp_header = Udp.MARKER_LINES
        udp_data = [
            Udp.FRAME_LIDAR,    # Frame (FRAME_LIDAR oder FRAME_MAP)
            Udp.NONE]           # Für Linien-Endpunkte keine Kreise zeichnen
            # Es folgen die Linien sx, sy, ex, ey ...
            # und danach die Farben der Linien
        cm = 100.0  # zur Umrechnung von Meter in cm
        udp_line_colors = []
    
        for start, end in walls:
            udp_line_colors.append(Udp.GREEN)
            sx, sy, ex, ey = round(start[0]*cm), round(start[1]*cm), round(end[0]*cm), round(end[1]*cm)
            udp_data.extend([sx, sy, ex, ey])
                
        if num_lines > 0:
            udp_data.extend(udp_line_colors)
            UdpSend(udp_header, udp_data)
            
        # UDP Kommando zum Zeichen von Punkten
        udp_header = Udp.MARKER_POINTS
        udp_data = [
            Udp.FRAME_LIDAR,    # Frame (FRAME_LIDAR oder FRAME_MAP)
            Udp.BLUE]           # Für Punkte blaue Kreise zeichnen (NONE = keine Kreise)
            # Es folgen die Punkte px1, py1, px2, py2 ...

        for p in punkte:
            px, py = round(p[0]*cm), round(p[1]*cm)
            udp_data.extend([px, py])

        UdpSend(udp_header, udp_data)
        
