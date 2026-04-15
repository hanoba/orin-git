import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import Ransac
from params import TaskState
from params import ReadYawOffset, WriteYawOffset


class YawOffsetDetectionTask:
    def Init(self, node, params, retvals=None):
        self.node = node
        self.node.SetWantedTheta(np.radians(90.0))
        self.numMeas = 0
        self.maxMeas = 10
        self.sumAngle_rad = 0.0
        self.yawOffsetOld = ReadYawOffset()

    def Step(self, scan_msg):
        if not self.node.wantedThetaReached:
            return TaskState.Running, None

        all_detected_walls = self.Walldetector(scan_msg) 
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
    
    def Walldetector(self, msg):
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        rangesMask = np.isfinite(ranges) & (ranges > msg.range_min + 0.01) & (ranges < msg.range_max - 0.1)
        points = np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))[rangesMask]
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
        pub = self.node.marker_pub
        markers = MarkerArray()
        frame_id = "lidar"
        z_height = +0.1
        
        # 1. Ein Marker für ALLE Linien
        m_lines = Marker()
        m_lines.header.frame_id = frame_id
        m_lines.header.stamp.sec = 0
        m_lines.header.stamp.nanosec = 0
        m_lines.ns = "walls_lines"
        m_lines.id = 0
        m_lines.type = Marker.LINE_LIST 
        m_lines.action = Marker.ADD
        m_lines.scale.x = 0.15
        m_lines.color.g = 1.0; m_lines.color.a = 0.8
        m_lines.pose.orientation.w = 1.0

        # 2. Ein Marker für ALLE Endpunkte (Sphären)
        m_spheres = Marker()
        m_spheres.header = m_lines.header
        m_spheres.ns = "walls_endpoints"
        m_spheres.id = 0
        m_spheres.type = Marker.SPHERE_LIST # <--- EXTREM EFFIZIENT
        m_spheres.action = Marker.ADD
        m_spheres.scale.x = m_spheres.scale.y =  m_spheres.scale.z = 0.25*2
        m_spheres.color.b = 1.0; m_spheres.color.a = 1.0

        for start, end in walls:
            p_start = Point(x=float(start[0]), y=float(start[1]), z=z_height)
            p_end = Point(x=float(end[0]), y=float(end[1]), z=z_height)

            wall_color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8) # if isDetectedWallValid[i] else ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)        
            
            # Punkte zur Linienliste hinzufügen
            m_lines.points.append(p_start)
            m_lines.points.append(p_end)

            # Farbe zweimal hinzufügen (für beide Enden des Segments)
            m_lines.colors.append(wall_color)
            m_lines.colors.append(wall_color)
            
        for p in punkte:
            # Punkt zur Sphärenliste hinzufügen
            sphere = Point(x=float(p[0]), y=float(p[1]), z=z_height)
            m_spheres.points.append(sphere)

        markers.markers.append(m_lines)
        markers.markers.append(m_spheres)
        
        pub.publish(markers)
