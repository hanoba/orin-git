import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
import math
import numpy as np # Vergiss nicht numpy zu importieren!
import sys
import ydlidar
import threading
from params import LidarMaxAngle


class eKarrenLidarNode(Node):
    def __init__(self):
        super().__init__('ekarren_lidar')
 
        # Custom Profil: Wir akzeptieren nur den EINEN neuesten Scan.
        # Alles was älter ist, wird sofort gelöscht.
        custom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Schnell, keine Garantien (wie UDP)
            history=HistoryPolicy.KEEP_LAST,
            depth=1,                                    # <--- WICHTIG: Puffergröße auf 1 zwingen!
            durability=DurabilityPolicy.VOLATILE)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', custom_qos)  

        # Lidar initialisieren
        self.LidarInit()

        # Eigener Thread für den Lidar da doProcessSimple blockiert!
        self.thread = threading.Thread(target=self.MainLoop, daemon=True)
        self.thread.start()

    def MainLoop(self):
        # Läuft permanent im Hintergrund und wartet auf neue Lidar-Scans.
        while rclpy.ok() and ydlidar.os_isOk():
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
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg() 
        msg.header.frame_id = 'lidar'
        
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
        
        # 7. ROS2 Message Parameter exakt auf unser Punkte-Format zuschneiden
        useLimitedRanges = True
        if useLimitedRanges:
            msg.angle_min = math.radians(-(LidarMaxAngle-1)) 
            msg.angle_max = math.radians(LidarMaxAngle)
            num_readings = 2*LidarMaxAngle
            msg.angle_increment = (msg.angle_max - msg.angle_min) / (num_readings - 1)
            msg.angle_max = msg.angle_min + (msg.angle_increment * (num_readings - 1))
        else:
            msg.angle_min = math.radians(0.0) 
            msg.angle_max = math.radians(359.0)
            msg.angle_increment = math.radians(1.0) # Exakt 1 Grad in Radiant
        
        # Konstanten vom Lidar übernehmen
        msg.time_increment = self.scan_data.config.time_increment
        msg.scan_time = self.scan_data.config.scan_time
        msg.range_min = self.scan_data.config.min_range
        msg.range_max = self.scan_data.config.max_range
        
        # Das berechnete NumPy-Array als Python-Liste übergeben
        msg.ranges = limited_ranges.tolist() if useLimitedRanges else min_ranges.tolist()
        
        # Intensities lassen wir leer, da wir das Minimum der Distanzen genommen haben 
        # und die Zuordnung zur Intensität jetzt nicht mehr eindeutig wäre.
        msg.intensities = [] 
        
        self.scan_pub.publish(msg)


def main():
    # ROS2 Initialisierung
    rclpy.init()
    node = eKarrenLidarNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Wird ausgelöst, wenn du Ctrl+C drückst
        node.get_logger().info("Node ekarren wird durch Benutzer abgebrochen...")
    except Exception as e:
        # Fängt unerwartete Fehler ab
        node.get_logger().error(f"Unerwarteter Fehler: {e}")
    finally:
        # Dieser Block wird IMMER ausgeführt, egal ob Fehler oder Ctrl+C
        node.StopLidar()
        node.get_logger().info("Bereinige Ressourcen...")
               
        # Nur zerstören, wenn der Kontext noch gültig ist
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        
        # Optional: Komplettes Beenden erzwingen (hilft bei WSL2-Hängern)
        sys.exit(0)

    
if __name__ == '__main__':
    main()