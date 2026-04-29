import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool
import math
import numpy as np # Vergiss nicht numpy zu importieren!
import sys
import ydlidar
import threading
from compass import Compass
from params import LidarMaxAngle
import socket
#from Rosmaster_Lib import Rosmaster     # for eKarren emulation with RosMaster X3 PLus

# Constants for eKarren
rcMaxValue = 2048
tauMax = 255
radAbstand = 0.68             # meter
vLinearMax = 1.63             # m/s
vAngularMax = 0.3*vLinearMax  # m/s
omegaMax_RadPerSec = 1.44  # rad/s vAngularMax_Hz*2*pi  

DEV_ROSMASTER = 0       # run natively on Rosmaster
DEV_EKARREN = 1         # send UDP commands to eKarren
DEV_EKARREN_PC = 2      # send UDP commands to PC (AZ-KENKO)
DEV_EKARREN_EMU = 3     # send UDP commands to Rosmaster


# Die Klasse eKarren stellt im wesentlichen ein Interface zum Setzen der Geschwindigkeit bereit.
# Weiterhin erlaubt die KLasse eine Emulation des eKarrens mit dem RosMaster X3 Plus. 
class eKarren:
    def __init__(self, device=DEV_EKARREN, debug=False):
        self.device = device
        #if self.device==DEV_ROSMASTER:
        #    # Roboter über USB-Port initialisieren
        #    self.rosmaster = Rosmaster(com="/dev/ttyCH341USB0", debug=debug)
        #    self.rosmaster.create_receive_threading()  # Empfangsthread für Statuswerte starten
        #
        #    # Kurze Pause für Initialisierung
        #    time.sleep(.1)
        #    # Startsignal: drei kurze Pieptöne
        #    for i in range(3):
        #        self.rosmaster.set_beep(60)
        #        time.sleep(.2)
        #    return

        if self.device==DEV_EKARREN_EMU:
            self.clientAddr = ("127.0.0.1", 4215)            # 
        elif self.device==DEV_EKARREN_PC:
            self.clientAddr = ("192.168.178.42", 4215)       # AZ-KENKO
        elif self.device==DEV_EKARREN:
            self.clientAddr = ("192.168.20.100", 4211)       # E-Karren
        else:
            print("Wrong device: {self.device}")
            sys.exit(1)

        self.rosmaster = None            
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.rcKeyStatus = 1
    
        # v = av*tau + bv
        av = 0.0176  
        bv = - 0.1348
        # tau = (v - bv)/av = at*v + bt
        self.bt = bv/av
        self.at = 1/av

    def GetVersion(self):
        if self.device==DEV_ROSMASTER: return self.rosmaster.get_version()
        return "V9"
        
    def GetBatteryVoltage(self):
        if self.device==DEV_ROSMASTER: return self.rosmaster.get_battery_voltage()
        return 24.0
        
    def CheckSum(self, send_data):
        checkSum = 0
        for i in range(len(send_data)):
            checkSum += ord(send_data[i])
        checkSum = checkSum & 255
        return checkSum
    
    def Quantize(self, x):
        y = int(round(x, 0))
        if y>= rcMaxValue: y = rcMaxValue - 1
        elif y <= -rcMaxValue: y = -rcMaxValue + 1
        return y
    
    # vLinear = linear Geschwindigkeit in m/s
    # omega = winkelgeschwindigkeit in rad/sec
    def SetSpeed(self, vLinear, omega):
        if self.device==DEV_ROSMASTER:
            # Steuerbefehl an RosMaster senden (Vorwärts-/Rückwärts- und Drehbewegung)
            # v_x=[-0.7, 0.7] m/s, v_y=[-0.7, 0.7] m/s, v_z=[-3.2, 3.2] rad/sec
            self.rosmaster.set_car_motion(v_x=-vLinear, v_y=0, v_z=-omega)
            return

        vLinearQ = self.Quantize(vLinear / vLinearMax * rcMaxValue)
        vAngular = -omega*radAbstand/2/10
        vAngularQ = self.Quantize(vAngular / vAngularMax * rcMaxValue)
        
        #print(f"{vLinear=}m/s  {vAngular=}m/s  {vLinearQ=}  {vAngularQ=}")
        send_data = f"AT+#,{vLinearQ},{vAngularQ},{self.rcKeyStatus}"
        send_data += f",0x{self.CheckSum(send_data):02X}"
        self.sock.sendto(send_data.encode('utf-8'), self.clientAddr)
 
    def Close(self):
        #sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
        self.SetSpeed(0, 0)
        if not self.device==DEV_ROSMASTER: self.sock.close()


class eKarrenNode(Node):
    def __init__(self):
        super().__init__('ekarren')

        # ROS2 Parameter "device" deklarieren (Name, Standardwert)
        self.declare_parameter('device', "eKarrenPC")
        deviceName = self.get_parameter('device').value
        if deviceName=="eKarren": deviceNum = DEV_EKARREN
        elif deviceName=="eKarrenPC": deviceNum = DEV_EKARREN_PC
        elif deviceName=="eKarrenEmulator": deviceNum = DEV_EKARREN_EMU
        else: 
            self.get_logger().error(f"Illegal deviceName: {deviceName} Valid: eKarren, eKarrenPC, eKarrenEmulator")
            sys.exit()
        self.get_logger().info(f"Device: {deviceName}")
  
        # Roboter initialisieren
        self.ekarren = eKarren(device=deviceNum, debug=False)
        self.vLinear = 0
        self.omega = 0

        # Custom Profil: Wir akzeptieren nur den EINEN neuesten Scan.
        # Alles was älter ist, wird sofort gelöscht.
        custom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Schnell, keine Garantien (wie UDP)
            history=HistoryPolicy.KEEP_LAST,
            depth=1,                                    # <--- WICHTIG: Puffergröße auf 1 zwingen!
            durability=DurabilityPolicy.VOLATILE)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', custom_qos)  

        # Subscriber für Fahrbefehle von ROS 2
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Compass initialisieren
        self.compass = Compass()

        # Publisher für Kompass (theta)
        self.theta_pub = self.create_publisher(Float32, '/compass_heading', custom_qos)
        self.lastTheta = 0.0
        self.yawCompass = 0.0

        # Höre auf das Topic des CompassCalibrationNode
        self.compassCalibrationRunning = False
        self.calib_sub = self.create_subscription(Bool, '/compass_calibration', self.calib_callback, 10)

        # Lidar initialisieren
        self.LidarInit()

        # Eigener Thread für den Lidar, Compass und Fahrbefehle, da doProcessSimple blockiert!
        self.thread = threading.Thread(target=self.MainLoop, daemon=True)
        self.thread.start()

        # --- TIMER FÜR 50 HZ ---
        # 0.02 Sekunden = 50 Hz. Diese Funktion ruft selbstständig timer_callback auf.
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info("50 Hz Timer gestartet. Node ist aktiv.")

    def timer_callback(self):
        self.yawCompass = self.compass.ReadYaw()
    
    def calib_callback(self, msg):
        # Wird automatisch aufgerufen, wenn der CompassCalibrationNode publiziert
        self.compassCalibrationRunning = msg.data
        self.get_logger().info(f"Neuer Status empfangen: compassCalibrationRunning={self.compassCalibrationRunning}")
           

    def MainLoop(self):
        # Läuft permanent im Hintergrund und wartet auf neue Lidar-Scans.
        # Sendet theta und Fahrbefehle synchronisiert mit Lidar-Scans (10Hz)
        while rclpy.ok() and ydlidar.os_isOk():
            # doProcessSimple blockiert hier solange, bis ein 360° Scan fertig ist
            if self.laser.doProcessSimple(self.scan_data):
                if not self.compassCalibrationRunning:
                    self.PublishTheta()
                self.PublishLidarData()
                self.ekarren.SetSpeed(self.vLinear, self.omega)
                    
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
    
    def cmd_vel_callback(self, msg):
        """Wird aufgerufen, wenn ROS2 einen Fahrbefehl sendet."""
        self.vLinear = msg.linear.x
        self.omega = msg.angular.z

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

    # Im Moment noch nur mit 10Hz
    def PublishTheta(self):
        theta = self.yawCompass
        if theta is not None: 
            self.lastTheta = theta
        msg = Float32()
        msg.data = float(self.lastTheta)
        self.theta_pub.publish(msg)

def main():
    # ROS2 Initialisierung
    rclpy.init()
    node = eKarrenNode()

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