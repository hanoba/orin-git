import math
import numpy as np # Vergiss nicht numpy zu importieren!
import sys
import ydlidar
import threading
from params import LidarMaxAngle, Udp
from UdpSend import UdpSend


class Lidar:
    def __init__(self, scanCallback):
        # ScanCallback-Funktion zum Publizieren von Lidar-Daten
        self.ScanCallback = scanCallback
        
        # Lidar initialisieren
        self.LidarInit()

        # Eigener Thread für den Lidar da doProcessSimple blockiert!
        self.thread = threading.Thread(target=self.MainLoop, daemon=True)
        self.thread.start()

    def MainLoop(self):
        # Läuft permanent im Hintergrund und wartet auf neue Lidar-Scans.
        
        # ydlidar.os_isOk() prüft, ob die Laufzeitumgebung des Treibers noch im 
        # ordnungsgemäßen Zustand ist und das Programm normal weiterlaufen kann.
        while ydlidar.os_isOk():
            # doProcessSimple blockiert hier solange, bis ein 360° Scan fertig ist
            if self.laser.doProcessSimple(self.scan_data):
                self.PublishLidarData()
                    
    def LidarInit(self):
        # === Parameter anpassen ===
        PORT = "/dev/ttyUSB0"
        BAUD = 512000           # TG30 nutzt 512000 Baud

        # === Lidar initialisieren ===
        self.laser = ydlidar.CYdLidar()
        self.laser.setlidaropt(ydlidar.LidarPropSerialPort, PORT)
        self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, BAUD)
        self.laser.setlidaropt(ydlidar.LidarPropFixedResolution, False)
        self.laser.setlidaropt(ydlidar.LidarPropReversion, False)
        self.laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
        self.laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TOF)
        self.laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)

        ret = self.laser.initialize()
        if ret:
            ret = self.laser.turnOn()
            if ret:
                self.get_logger().info(f"YDLidar TG30 erfolgreich auf {PORT} gestartet!")
            else:
                self.get_logger().error("Lidar Motor konnte nicht gestartet werden.")
        else:
            self.get_logger().error(f"Lidar auf {PORT} nicht gefunden. Rechte geprüft (dialout)?")

        self.scan_data = ydlidar.LaserScan()

    def StopLidar(self):
        # Den Scan-Vorgang stoppen (schaltet Laser und Motor ab)
        self.laser.turnOff() 
        
        # Verbindung trennen
        self.laser.disconnecting()

    def PublishLidarData(self):
        # 1. Listen-Comprehension für Rohdaten (die einzige Python-Schleife, da p ein C++ Struct ist)
        # Wir gehen davon aus, dass p.angle in Radiant vorliegt
        # Wir setzen ein Minus vor p.angle, um die Drehrichtung umzukehren 
        # und addieren pi/2 um den Einbauwinkel zu korrigieren.
        angles_rad = np.array([(np.pi/2-p.angle) % (2*np.pi) for p in self.scan_data.points])
        ranges_raw = np.array([p.range for p in self.scan_data.points])
        
        # 2. Winkel in Grad umrechnen und dem passenden 1°-Bucket (0-359) zuordnen
        # np.floor rundet ab (z.B. 14.8° -> 14°). Modulo 360 fängt negative Winkel/Überläufe ab.
        buckets = np.floor(np.degrees(angles_rad)).astype(int) % 360
        
        # 3. Ziel-Array für 360 Grad anlegen. 
        # Mit np.inf (Unendlich) füllen, da wir später das Minimum suchen.
        min_ranges = np.full(360, np.inf)
        
        # 4. Filter: 0.0 bedeutet beim Lidar meist "Kein Echo / Fehler". 
        # Diese dürfen nicht das Minimum werden!
        valid_mask = ranges_raw > 0.0
        valid_buckets = buckets[valid_mask]
        valid_ranges = ranges_raw[valid_mask]
        
        # 5. DIE NUMPY-MAGIC:
        # Geht alle valid_buckets durch und trägt in 'min_ranges' den kleinsten 
        # Wert aus 'valid_ranges' ein, der auf diesen Index (0-359) fällt.
        #
        # Stell dir vor, du hast mehrere Pakete (Lidar-Punkte), die in denselben Eimer (bucket) geworfen werden sollen. 
        # np.minimum.at sorgt dafür, dass am Ende nur das kleinste Paket im Eimer liegen bleibt.
        # min_ranges:    Das Array, das die Ergebnisse speichern soll (deine "Eimer" mit np.inf vorinitialisiert). 
        # valid_buckets: Ein Array von Indizes. Es sagt: "Der Wert an Stelle i aus valid_ranges gehört in den Eimer Nummer X.
        # valid_ranges:  Die tatsächlichen Messwerte (Entfernungen), die einsortiert werden sollen.
        np.minimum.at(min_ranges, valid_buckets, valid_ranges)
        
        # 6. Unveränderte Werte (wo kein einziger Lidar-Punkt reinfiel) auf 0.0 setzen
        min_ranges[np.isinf(min_ranges)] = 0.0
        
        # Winkel auf -180° bis +179° normieren und min_ranges entsprechend umsortieren
        # Erzeugt Werte von 0 bis 180 und von -179 bis -1
        angles = np.concatenate([np.arange(0, 181), np.arange(-179, 0)])
        sort_indices = np.argsort(angles)
        sorted_ranges = min_ranges[sort_indices]

        # --- Begrenzung auf -(LidarMaxAngle-1) ... LidarMaxAngle ---
        # Wir schneiden die entsprechenden Indizes aus
        start_idx = 179 - (LidarMaxAngle - 1)
        end_idx = 180 + LidarMaxAngle
        limited_ranges = sorted_ranges[start_idx:end_idx]       
        self.ScanCallback(limited_ranges)
        
        cm = 100.0
        ranges_cm = limited_ranges*cm        
        ranges_cm = ranges_cm.astype(np.int16)
        UdpSend(Udp.LIDAR_DATA, ranges_cm.tolist())

