#!/usr/bin/env python3

import platform
import math
import numpy as np
import sys
import socket
import select
import struct
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore, QtGui

# angle range = -180 ... 180 = -LidarMaxAngle ... LidarMaxAngle
LidarMaxAngle = 180


#if platform.node() != "AZ-KENKO":
##    import ydlidar
##    
##    class Lidar():
##        def __init__(self):
##            # === Parameter anpassen ===
##            PORT = "/dev/ttyUSB0"
##            BAUD = 512000           # TG30 nutzt 512000 Baud
##    
##            # === Lidar initialisieren ===
##            self.laser = ydlidar.CYdLidar()
##            self.laser.setlidaropt(ydlidar.LidarPropSerialPort, PORT)
##            self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, BAUD)
##            self.laser.setlidaropt(ydlidar.LidarPropFixedResolution, False)
##            self.laser.setlidaropt(ydlidar.LidarPropReversion, False)
##            self.laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
##            self.laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TOF)
##            self.laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
##    
##            ret = self.laser.initialize()
##            if not ret:
##                print("Fehler: Initialisierung fehlgeschlagen")
##                exit(1)
##    
##            if not self.laser.turnOn():
##                print("Fehler: Laser konnte nicht gestartet werden")
##                exit(1)
##    
##            print("LIDAR läuft – Strg+C zum Beenden")
##            self.scan = ydlidar.LaserScan()
##    
##    
##        def Scan(self):
##            if self.laser.doProcessSimple(self.scan):
##                # Rohdaten in Winkel/Entfernung umrechnen
##                #ang270 = 270/180*np.pi
##                #angles =  np.array([ang270 - p.angle for p in self.scan.points])
##                #angles = (angles + np.pi) % (2*np.pi) - np.pi
##                #angles =  np.array([(ang270 - p.angle + np.pi) % (2*np.pi) - np.pi for p in self.scan.points])
##                # Umrechnung in Roboterkoordinationsystem (x zeigt in Fahrtrichtung
##                #angles =  np.array([(np.pi - p.angle + np.pi) % (2*np.pi) - np.pi for p in self.scan.points])
##                angles =  np.array([(np.pi/2 - p.angle) % (2*np.pi) for p in self.scan.points])
##    
##                radius  = np.array([p.range for p in self.scan.points])
##        
##                return angles, radius
##                
##            else:
##                time.sleep(0.05)
##                return [], []
##                
##        def Close(self):
##            # === Aufräumen ===
##            self.laser.turnOff()
##            self.laser.disconnecting()
#else:

class Lidar:
    def __init__(self, ip="127.0.0.1", port=5005):
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
        #print(f"Höre auf UDP-Daten unter {ip}:{port}...")
        
        # Winkel erzeugen von 0° bis 359° (die Winkel sind immer gleich)
        angles = np.deg2rad(np.arange(360, dtype=float))
        
        # umsortieren von 0° bis 359° auf -180° bis 179°
        self.angles_rad = np.concatenate((angles[360-LidarMaxAngle:360]-2*np.pi, angles[0:LidarMaxAngle]))
        
        # Odometry Data
        self.posX = 0.0
        self.posY = 0.0
        self.theta_deg = 0.0 
        self.sim_time_sec = 0.0
        
        self.InitLidarData()

    def InitLidarData(self):
        #  range value per degree
        self.dist_mm = np.zeros(360, dtype=np.uint16) + 65535
        self.numPoints = 0
            
    def UnpackLidarData(self, byte_packet):
        # Header-Größe definieren
        # 1x Float (8 Byte) 2x Float (4 Byte) + 1x Unsigned Int (4 Byte) = 20 Bytes
        header_size = struct.calcsize("<dffI")
        
        # Den Header extrahieren
        header_part = byte_packet[:header_size]
        time, angle_min, angle_max, num_elements = struct.unpack("<dffI", header_part)
        print(f"{time=}, {angle_min=}, {angle_max=}, {num_elements=}")
        
        # Die Ranges extrahieren
        ranges_part = byte_packet[header_size:]
        
        # Sicherheitscheck: Entspricht die restliche Byte-Länge der erwarteten Anzahl?
        expected_range_bytes = num_elements * 2
        if len(ranges_part) != expected_range_bytes:
            raise ValueError("Byte-Paket hat nicht die angegebene Anzahl an Elementen!")

        # Wir wandeln die Bytes direkt in ein NumPy-Array um (extrem schnell)
        ranges_mm = np.frombuffer(ranges_part, dtype=np.uint16)

        # Werte im Grad-Raster speichern
        angle = angle_min
        angle_inc = (angle_max - angle_min) / (num_elements - 1)
        for i in range(num_elements):
            ang = int(round(angle))
            if ang==360: ang = 0
            self.dist_mm[ang] = min(ranges_mm[i], self.dist_mm[ang])
            angle += angle_inc
            
        self.numPoints += num_elements
        if self.numPoints >= 360*4:
            dist_mm_copy = self.dist_mm.copy()
            self.InitLidarData()
            return dist_mm_copy
        return None

    def GetScan(self):
        """Prüft auf neue Daten ohne zu blockieren."""
        if self.is_closed:
            return None
            
        try:
            # Überprüfung des Dateideskriptors
            if self.sock.fileno() == -1:
                return None
                
            # select prüft den Puffer
            ready = select.select([self.sock], [], [], 0)
            if ready[0]:
                data, addr = self.sock.recvfrom(1024)

                # (3 * 4 Bytes für Floats) + (1 * 8 Bytes für double) = 20 Bytes
                ODOMETRY_PACKET_SIZE = 20

                if len(data) == ODOMETRY_PACKET_SIZE:
                    # 2. Odometrie entpacken (< = Little Endian, fff = 3x Float, d = 1x Double)
                    self.posX, self.posY, self.theta_deg, self.sim_time_sec = struct.unpack('<fffd', data)
                else:
                    radius = self.UnpackLidarData(data)
                    if radius == None: 
                        return None
                        
                    # umsortieren von 0° bis 360° auf -180° bis 180°
                    dist_mm = np.concatenate((radius[360-LidarMaxAngle:360], radius[0:LidarMaxAngle]))                    
                    dist = dist_mm.astype(float) / 1000.0 # mm -> Meter
                    
                    return self.angles_rad, dist, self.posX, self.posY, self.theta_deg, self.sim_time_sec
        except (OSError, ValueError, AttributeError):
            return None
        return None

    def close(self):
        """Beendet die Verbindung sauber."""
        if not self.is_closed:
            self.is_closed = True
            try:
                if self.sock.fileno() != -1:
                    self.sock.close()
            except:
                pass


class LidarApp(QtWidgets.QMainWindow):
    def __init__(self, processLidarData):
        super().__init__()

        self.processLidarData = processLidarData

        # Wir brauchen einen Container, um mehrere Dinge (Plot + Text) anzuzeigen
        self.container = QtWidgets.QWidget()
        self.setCentralWidget(self.container)

        # Layout erstellen und an den Container binden
        self.layout = QtWidgets.QVBoxLayout(self.container) 
        
        # Das Plot-Widget (Diagramm)
        self.setWindowTitle("Lidar Live-Scan")
        self.view = pg.PlotWidget(title="Entfernung in Metern (Beenden mit ESC)")
        self.layout.addWidget(self.view)
        
        #self.setCentralWidget(self.view)
        
        # Die Textzeile (Status-Label) hinzufügen
        self.status_label = QtWidgets.QLabel("Initialisiere Lidar...")
        self.status_label.setStyleSheet("color: white; background-color: black; font-size: 14px; padding: 5px;")
        self.layout.addWidget(self.status_label)        
        
        # Design & Skalierung
        self.view.setAspectLocked(True)
        self.view.setXRange(-12, 12)
        self.view.setYRange(-12, 12)
        self.view.showGrid(x=True, y=True)
        
        # Plot-Element (Grüne Punkte)
        self.plot1 = self.view.plot(pen=None, symbol='o', symbolSize=4, symbolBrush=(0, 255, 0))
        self.plot2 = self.view.plot(pen=None, symbol='o', symbolSize=8, symbolBrush=(255, 0, 0))
        
        self.lidar = Lidar()
        
        # Haupt-Timer für Grafik-Updates (ca. 50 FPS)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(20)
        
        # ESC-Shortcut als primärer Abbruchweg
        self.shortcut_close = QtWidgets.QShortcut(QtGui.QKeySequence("Escape"), self)
        self.shortcut_close.activated.connect(self.close)
        
        #self.showFullScreen()
        self.showNormal()

    def update_plot(self):
        if self.lidar.is_closed:
            return
            
        retvals = self.lidar.GetScan()
        if retvals is None:
            return
            
        angles, radius, posX, posY, yaw, time = retvals 
        # Transformation Polar -> Kartesisch
        x1 = radius * np.cos(angles)
        y1 = radius * np.sin(angles)
        if len(x1) > 0:
            x2, y2 = self.processLidarData(angles, radius)
            self.plot1.setData(x1, y1)
            self.plot2.setData(x2, y2)

            # Textzeile aktualisieren (Beispiel: Zeigt die Position des roten Punkts)       
            self.status_label.setText(f"{time:8.2f} {posX=:.2f}m {posY=:.2f}m {yaw=:.0f}°")

    def closeEvent(self, event):
        """Aufräumen beim Schließen des Fensters."""
        self.timer.stop()
        self.lidar.close()
        print("Lidar-Verbindung getrennt. Anwendung beendet.")
        event.accept()
