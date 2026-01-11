#!/usr/bin/env python3
import numpy as np
import math
import socket
import select
import struct
from params import LidarMaxAngle

#lidarX=np.zeros(360*4)
#lidarY=np.zeros(360*4)

class Lidar():
    def __init__(self, lidar, measPerDeg, backWheelDrive, udpIp=""):
        self.lidar = lidar
        self.maxDist_mm = 10000
        self.dist_mm = np.zeros(360, dtype=np.uint16) + self.maxDist_mm
        #self.angles = np.zeros(360*measPerDeg) + 400.0
        self.measPerDeg = measPerDeg
        self.angOffset = 180 if backWheelDrive else 0      # 0°=Front 180°=Back
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
        self.debugCntMax = 10
        self.totalPoints = 0
        self.Debug(f"Lidar sensor initialized")
        
    def Debug(self, text):
        if self.debugCnt < self.debugCntMax:
            print(f"[Lidar.Debug] {text}")
        
    def UdpSendLidarData(self):
        # send self.dist_mm via udp
        if self.udpIp != "":
            # Den Lidar-Teil direkt als Bytes nehmen (360 * 2 Bytes = 720 Bytes)
            lidar_bytes = self.dist_mm.tobytes()
            self.sock.sendto(lidar_bytes, (self.udpIp, self.udpPort))                

        
    def UdpSendPositionAndTime(self, posX, posY, yaw, time):
        # send self.dist_mm via udp
        if self.udpIp != "":
            # Die Odometrie-Floats und Time (Double) packen (3*4 + 1*8 Bytes = 20 Bytes)
            odometry_bytes = struct.pack('<fffd', posX, posY, yaw, time)
            self.sock.sendto(odometry_bytes, (self.udpIp, self.udpPort))                

    def GetDistArray(self, posX, posY, yaw, time):
        self.debugCnt += 1
        
        # Send Postion and Time
        self.UdpSendPositionAndTime(posX, posY, yaw, time)
        
        # LiDAR-Frame holen
        frame = self.lidar.get_current_frame()
        if frame is None:
            return []

        new_step = frame["physics_step"]
        if new_step == self.last_step:
            return []
        self.last_step = new_step
        
        self.frameCnt += 1
        # skip first dummy frame
        if self.frameCnt == 0:
            return []
            
        global lidarX, lidarY
        pc = frame["point_cloud"].reshape(-1, 3)   # shape (N, 1, 3) – Punkte im Lidar/Robot-Frame

        angMin =  99000
        angMax = -99000
        
        numPoints = len(pc[:, 0])
        for i in range(numPoints):
            x = pc[i, 0]
            y = pc[i, 1]

            ang = math.degrees(math.atan2(y, x))
            ang = (ang + self.angOffset) % 360.0      # Lidar 180° gedreht
            ang = int(round(ang)) 
            if i==0: ang0=ang
            elif i==numPoints-1: ang239=ang
            
            self.totalPoints += 1
            if ang < angMin: angMin = ang
            elif ang > angMax: angMax = ang

            if ang==360: ang = 0

            r = math.hypot(x, y)
            r = r if r < self.maxDist_mm else self.maxDist_mm
            r = int(round(1000*r))     # Entfernung in mm            
            self.dist_mm[ang] = min(r, self.dist_mm[ang])
            #lidarX[self.totalPoints] = pc[i, 0]
            #lidarY[self.totalPoints] = pc[i, 1]

        self.Debug(f"{self.last_step:6d}: {numPoints=}  {angMin=:6.2f}  {angMax=:6.2f}     {ang0=:6.2f}  {ang239=:6.2f}")


        # Zum richtigen Zeitpunkt senden damit der Abtastzeitpunkt des Bereichs
        # von -LidarMaxAngle bis +LidarMaxAngle kontinuierlich ist
        #if angMax == LidarMaxAngle and self.totalPoints >= 360*self.measPerDeg:

        if self.totalPoints >= 360*self.measPerDeg:
            self.totalPoints = 0
            distCopy = self.dist_mm.copy() / 1000
            self.UdpSendLidarData()
            self.dist_mm.fill(self.maxDist_mm)
            return distCopy

        return []

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