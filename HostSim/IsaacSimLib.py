#!/usr/bin/env python3
import numpy as np
import math
import socket
import select
import struct

#lidarX=np.zeros(360*4)
#lidarY=np.zeros(360*4)

class UdpBridge():
    def __init__(self, lidar, measPerDeg, backWheelDrive, udpIp=""):
        self.lidar = lidar
        self.maxDist = 60.
        #self.dist_mm = np.zeros(360, dtype=np.uint16) + self.maxDist_mm
        self.measPerDeg = measPerDeg
        self.backWheelDrive = backWheelDrive
        #self.angOffset = 180 if backWheelDrive else 0      # 0°=Front 180°=Back
        self.udpIp = udpIp      # localhost = "127.0.0.1"
        self.udpPort = 5005
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.Init()

    def Init(self):
        self.lidar.initialize()                   # wichtig
        self.lidar.add_point_cloud_data_to_frame()
        self.lidar.enable_visualization()
        self.frameCnt=-1
        self.last_step = -1
        self.debugCnt = 0
        self.debugCntMax = 1000
        self.totalPoints = 0
        
    def Debug(self, text):
        if self.debugCnt < self.debugCntMax:
            print(f"[Lidar.Debug] {text}")
        
    def OldUdpSendLidarData(self):
        # send self.dist_mm via udp
        if self.udpIp != "":
            # Den Lidar-Teil direkt als Bytes nehmen (360 * 2 Bytes = 720 Bytes)
            lidar_bytes = self.dist_mm.tobytes()
            self.sock.sendto(lidar_bytes, (self.udpIp, self.udpPort))                
        
    def UdpSendLidarData(self, time, angle_min, angle_max, ranges_mm):
        # send angle_min, angle_max, ranges_mm via udp 
        if self.udpIp != "":
            # 2. Header packen:
            # 'f' = float (4 bytes)
            # 'I' = unsigned int (4 bytes)
            # '<' = Little-Endian (Standard für die meisten CPUs/ROS)
            num_elements = len(ranges_mm)
            header_bytes = struct.pack("<dffI", time, angle_min, angle_max, num_elements)

            # 3. Ranges packen:
            # 'H' = unsigned short / uint16 (2 bytes)
            # Wir erstellen ein Format-String wie "240H" für 240 uint16 Werte
            ranges_bytes = struct.pack(f"<{num_elements}H", *ranges_mm)

            # 4. Alles zu einem Byte-Array zusammenfügen
            full_packet = header_bytes + ranges_bytes
            
            # Senden über UDP
            self.sock.sendto(full_packet, (self.udpIp, self.udpPort)) 
        
    def UdpSendPositionAndTime(self, posX, posY, yaw, time):
        # send self.dist_mm via udp
        if self.udpIp != "":
            # Die Odometrie-Floats und Time (Double) packen (3*4 + 1*8 Bytes = 20 Bytes)
            odometry_bytes = struct.pack('<fffd', posX, posY, yaw, time)
            self.sock.sendto(odometry_bytes, (self.udpIp, self.udpPort))                

    def print_frame_structure(self, data):
        self.Debug("\n--- FRAME STRUKTUR ---")
        for key, value in data.items():
            if isinstance(value, np.ndarray):
                # Bei Arrays: Zeige Shape (Dimensionen) und Datentyp
                self.Debug(f"Key: '{key:15}' | Type: np.ndarray | Shape: {value.shape} | Dtype: {value.dtype}")
            elif isinstance(value, list):
                # Bei Listen: Zeige Länge
                self.Debug(f"Key: '{key:15}' | Type: list       | Len:   {len(value)}")
            else:
                # Bei einfachen Werten (float, int, string): Zeige den Wert direkt
                self.Debug(f"Key: '{key:15}' | Type: {type(value).__name__:<10} | Wert:  {value}")
        self.Debug("----------------------\n")


    def SendData(self, posX, posY, yaw, time):
        self.debugCnt += 1
        
        # Send Postion and Time
        self.UdpSendPositionAndTime(posX, posY, yaw, time)
        
        # LiDAR-Frame holen:
        # FRAME STRUKTUR:
        # Key: 'time'         | Type: float
        # Key: 'physics_step' | Type: int
        # Key: 'point_cloud'  | Type: np.ndarray | Shape: (240, 1, 3)
        frame = self.lidar.get_current_frame()
        if frame is None:
            return

        #self.print_frame_structure(frame)

        new_step = frame["physics_step"]
        if new_step == self.last_step:
            return
        self.last_step = new_step
        
        self.frameCnt += 1
        # skip first dummy frame
        if self.frameCnt == 0:
            return
            
        #global lidarX, lidarY

        # Punktwolke holen und umformen
        pc = frame["point_cloud"].reshape(-1, 3)   # shape (N, 1, 3) – Punkte im Lidar/Robot-Frame

        #angMin =  99000
        #angMax = -99000
        #
        numPoints = len(pc[:, 0])
        assert numPoints == self.measPerDeg*60
        #for i in range(numPoints):
        #    x = pc[i, 0]
        #    y = pc[i, 1]
        #
        #    ang = math.degrees(math.atan2(y, x))
        #    ang = (ang + self.angOffset) % 360.0      # Lidar 180° gedreht
        #    ang = int(round(ang)) 
        #    if i==0: ang0=ang
        #    elif i==numPoints-1: ang239=ang
        #    
        #    self.totalPoints += 1
        #    if ang < angMin: angMin = ang
        #    elif ang > angMax: angMax = ang
        #
        #    if ang==360: ang = 0
        #
        #    r = math.hypot(x, y)
        #    r = r if r < self.maxDist_mm else self.maxDist_mm
        #    r = int(round(1000*r))     # Entfernung in mm            
        #    self.dist_mm[ang] = min(r, self.dist_mm[ang])

        # time holen
        time = frame["time"]
        
        # Winkel berechnen
        # Normaler arctan2 liefert Winkel von -pi bis +pi
        angles = np.degrees(np.arctan2(pc[:, 1], pc[:, 0]))
        
        # Handling of backWheelDrive: 180° (Pi Radiant) drehen
        if self.backWheelDrive:
            # Wir addieren Pi zu jedem Winkel. 
            angles = angles + 180.0

        # Wir verwenden Winkel im Bereich 0 bis 2*pi (np.tau = 2*pi)
        angles = angles % 360.0
        
        # Distanz berechnen (in Metern)
        ranges_m = np.linalg.norm(pc[:, :2], axis=1)
        
        # Clipping und Konvertierung in mm und uint16
        ranges_mm = np.clip(ranges_m * 1000.0, 0, self.maxDist).astype(np.uint16)
        
        # Sortieren (wichtig, damit die Winkel-Reihenfolge stimmt)
        sorted_indices = np.argsort(angles)
        
        dist_mm = ranges_mm[sorted_indices]
        angMin = float(angles[sorted_indices][0])
        angMax = float(angles[sorted_indices][-1])

        self.Debug(f"{time=:6.3f}  {numPoints=}  {angMin=:6.2f}  {angMax=:6.2f}")
        self.UdpSendLidarData(time, angMin, angMax, ranges_mm)
        
        #if self.totalPoints >= 360*self.measPerDeg:
        #    self.totalPoints = 0
        #    distCopy = self.dist_mm.copy() / 1000
        #    self.UdpSendLidarData()
        #    self.dist_mm.fill(self.maxDist_mm)
        #    return distCopy

        return

CMD_VELOCITY = 1

class RobotCtrl:
    def __init__(self, ip="127.0.0.1", port=5006):
        self.udpIp = ip
        self.udpPort = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind((self.udpIp, self.udpPort))
        except Exception as e:
            print(f"Fehler beim Binden des Sockets: {e}")
            sys.exit(1)
            
        self.sock.setblocking(False)
        self.is_closed = False
        print(f"Höre auf UDP-Daten unter {ip}:{port}...")

    def GetCmd(self):
        """Prüft auf neue Kommandos ohne zu blockieren."""
        if self.is_closed:
            return None, None
            
        try:
            # Überprüfung des Dateideskriptors
            if self.sock.fileno() == -1:
                return None, None
                
            # select prüft den Puffer
            ready = select.select([self.sock], [], [], 0)
            if ready[0]:
                data_bytes, _ = self.sock.recvfrom(1024)
                
                # Konvertierung: 4 Bytes pro Wert (360 Werte = 720 Bytes)
                cmd = np.frombuffer(data_bytes, dtype=float)
                cmdLen = len(cmd)
                assert cmdLen > 0
                #print(f"UDP command: {cmd}")
                
                cmdType = int(cmd[0])
                return cmdType, cmd[1:]
        except (OSError, ValueError, AttributeError):
            return None, None
        return None, None

    def close(self):
        """Beendet die Verbindung sauber."""
        if not self.is_closed:
            self.is_closed = True
            try:
                if self.sock.fileno() != -1:
                    self.sock.close()
            except:
                pass


# --- Beispiel für die Verwendung in deiner Isaac-Schleife ---
# calc = OdometryCalculator()
# ... in der Schleife:
# dx, dy, dtheta = calc.compute_delta(sim_x_mm, sim_y_mm, sim_yaw_rad)
# send_to_udp(lidar_data, dx, dy, dtheta)