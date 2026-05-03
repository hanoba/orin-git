import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool
import sys
from compass import Compass
import socket


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
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, custom_qos)

        # Compass initialisieren
        self.compass = Compass()

        # Publisher für Kompass (theta)
        self.theta_pub = self.create_publisher(Float32, '/compass_heading', custom_qos)
        self.lastTheta = 0.0

        # Höre auf das Topic des CompassCalibrationNode
        self.compassCalibrationRunning = False
        self.calib_sub = self.create_subscription(Bool, '/compass_calibration', self.calib_callback, 10)

        # --- TIMER FÜR 50 HZ ---
        # 0.02 Sekunden = 50 Hz. Diese Funktion ruft selbstständig timer_callback auf.
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info("50 Hz Timer gestartet. Node ist aktiv.")

    def timer_callback(self):
        # Sendet theta und Fahrbefehle (50Hz)
        if not self.compassCalibrationRunning:
            self.PublishTheta()
        self.ekarren.SetSpeed(self.vLinear, self.omega)
    
    def calib_callback(self, msg):
        # Wird automatisch aufgerufen, wenn der CompassCalibrationNode publiziert
        self.compassCalibrationRunning = msg.data
        self.get_logger().info(f"Neuer Status empfangen: compassCalibrationRunning={self.compassCalibrationRunning}")

    def cmd_vel_callback(self, msg):
        """Wird aufgerufen, wenn ROS2 einen Fahrbefehl sendet."""
        self.vLinear = msg.linear.x
        self.omega = msg.angular.z

    # Im Moment noch nur mit 10Hz
    def PublishTheta(self):
        theta = self.compass.ReadYaw()
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
        node.get_logger().info("Bereinige Ressourcen...")
               
        # Nur zerstören, wenn der Kontext noch gültig ist
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        
        # Optional: Komplettes Beenden erzwingen (hilft bei WSL2-Hängern)
        sys.exit(0)

    
if __name__ == '__main__':
    main()