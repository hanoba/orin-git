import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32
import numpy as np
import time
import sys
import math
from Ransac import LineDetection, PublishMarkers

sys.path.append('../HostSim')
import params

ThetaWallNorth_rad     =  0.00000    #    0.0°
ThetaWallEast_rad      =  1.48406    #   85.0°
ThetaWallSouth_rad     = -0.36485    #  -20.9°
ThetaWallWest_rad      =  1.45702    #   83.5°
ThetaSchuppenEast_rad  =  np.pi/2    #   90.0°
ThetaSchuppenSouth_rad =  0.00000    #    0.0°

LenSchuppenSouth =  4.00
LenSchuppenEast =  4.50
LenGartenhausSouth =  7.23
LenGartenhausEast =  4.67
LenTerrasseSouth =  6.08
LenTerrasseEast =  4.45
LenBassinSouth =  2.40
LenBassinEast =  2.80
LenSchuppenSouth =  4.00
LenSchuppenEast =  4.50
LenGartenhausSouth =  7.23
LenGartenhausEast =  4.67
LenTerrasseSouth =  6.08
LenTerrasseEast =  4.45
LenBassinSouth =  2.40
LenBassinEast =  2.80

STATE_INIT        = 0
STATE_CHECK_NORTH = 1
STATE_CHECK_SOUTH = 2

def WinkelNorm(angle_rad):
    return (angle_rad + math.pi) % math.tau - np.pi

def CheckWinkel(winkel_rad, value_rad):
    MaxDiff = np.deg2rad(5.0)
    diff1 = WinkelNorm(winkel_rad - value_rad)
    diff2 = WinkelNorm(winkel_rad + math.pi - value_rad)
    return abs(diff1) < MaxDiff or abs(diff2) < MaxDiff 

def CheckLength(vektorLen, value):
    Margin = 0.50
    if vektorLen < value - Margin: return False
    if vektorLen > value + Margin: return False
    return True

def DistY(start, end):
    # Berechne die parameter a und b der Geraden y=a*x+b, die durch die Punkte start und end geht
    a = (end[1] - start[1]) / (end[0] - start[0])
    b = start[1] - a*start[0]
    return b


class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self.declare_parameter('distance_threshold', 0.05) 
        self.declare_parameter('min_points', 6)
        # NEU: Wenn Punkte weiter als 50cm auseinander liegen -> Tor/Lücke
        self.declare_parameter('max_gap', 5.50) 

        #self.sub = self.create_subscription(LaserScan, '/scan', self.ScanCallback, qos_profile_sensor_data)
        # 1. Wir definieren ein striktes Profil: "Ich will nur das Allerneueste!"
        qos_policy = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # Wichtig: Muss zum Lidar passen (meist Best Effort)
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # <--- DER ENTSCHEIDENDE PUNKT: Queue-Größe = 1
        )
        
        # 2. Subscription mit diesem Profil erstellen
        self.sub = self.create_subscription(LaserScan, '/scan', self.ScanCallback, qos_policy)
        
        self.marker_pub = self.create_publisher(MarkerArray, '/garden_features', 10)
        
        # Erstellen der Subscription
        # 1. Typ: Float32 (muss zum Publisher passen)
        # 2. Topic: '/compass_heading'
        # 3. Callback: self.listener_callback
        # 4. QoS: qos_profile_sensor_data (MUSS zum Publisher passen)
        self.subCompass = self.create_subscription(
            Float32,
            '/compass_heading',
            self.CompassCallback,
            qos_profile_sensor_data
        )
        self.is_processing = False
        self.max_gap = self.get_parameter('max_gap').value
        self.theta = 0.0
        self.wantedTheta = 0.0
        self.wantedThetaReached = False
        self.K_head = 1.0
        self.omegaMax = 2.0
        self.state = STATE_INIT
        self.counter = 0
                
        # Publisher für die Fahrbefehle
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
    def SetWantedTheta(self, wantedTheta):
        self.wantedTheta = wantedTheta
        self.wantedThetaReached = False
        print(f"{self.wantedTheta=}")

    def CompassCallback(self, msg):
        self.theta = (msg.data + math.pi) % math.tau - np.pi
        e = self.wantedTheta - self.theta
        e = (e + math.pi) % math.tau - np.pi
        e = np.clip(e, -self.omegaMax, self.omegaMax)
        #print(f"{self.theta=}  {e=}  {self.wantedTheta=}")        
        omega = 0.0
        if abs(e) < 0.002 and not self.wantedThetaReached: 
            self.wantedThetaReachedTime = self.get_clock().now().nanoseconds / 1e9
            self.wantedThetaReached = True
            print("wantedThetaReached")
        elif not self.wantedThetaReached: 
            omega = e * self.K_head
        drive_msg = Twist()
        drive_msg.linear.x = float(0.0)
        drive_msg.angular.z = float(omega)
        self.cmd_pub.publish(drive_msg)

    def ScanCallback(self, msg):
        if self.is_processing: return
        self.is_processing = True
        self.StateMachine(msg)
        self.is_processing = False

    def Walldetector(self, msg):
        start_zeit = time.perf_counter() # Zeitnahme startet
        ranges = np.array(msg.ranges)
        valid = np.isfinite(ranges) & (ranges > params.LidarRangeMin + 0.01) & (ranges < params.LidarRangeMax - 0.1)
        #valid = np.isfinite(ranges) & (ranges > 0.1) & (ranges < 30.0)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        
        points = np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))[valid]

        all_detected_walls = LineDetection(points)
        now = self.get_clock().now().to_msg()
        #PublishMarkers(now, self.marker_pub, all_detected_walls)
        #self.PublishMarkers(all_detected_walls)
        
        ende_zeit = time.perf_counter() # Zeitnahme endet
        dauer_ms = (ende_zeit - start_zeit) * 1000 # Umrechnung in Millisekunden
        
        # Ausgabe im Terminal (alle Sekunde, um das Terminal nicht zu fluten)
        self.get_logger().info(f"⏱️ Rechenzeit WallDetector: {dauer_ms:.2f} ms  Theta={np.rad2deg(self.theta):.0f}°", throttle_duration_sec=1.0)
        return all_detected_walls

    
    def Localization(self, theta, all_detected_walls):
        theta_deg = np.rad2deg(theta)
        c, s = np.cos(theta), np.sin(theta)
        # 3. Rotationsmatrix erstellen
        # Formel: [[cos, -sin], [sin, cos]]
        # Da unsere Punkte Zeilenvektoren sind (N,2), müssen wir 
        # die Matrix für die Multiplikation transponieren oder die Formel anpassen.
        rotation_matrix = np.array([
            [c, -s],
            [s,  c]
        ])
        a = np.array(all_detected_walls)
        self.get_logger().info(f"all_detected_walls: {a.shape=}  {a.dtype=} {a.ndim=} {a.size=}", throttle_duration_sec=1.0)
        for start, end in a:   # all_detected_walls:
            # Drehung durch Matrix-Multiplikation (Dot Product)
            # Wir transponieren die Matrix (.T), damit (N,2) * (2,2) funktioniert        
            start = np.dot(start, rotation_matrix.T)
            end = np.dot(end, rotation_matrix.T)
            vektor = end - start
            vektorLen = np.linalg.norm(vektor)
            winkel_rad = np.arctan2(vektor[1], vektor[0])
            print(f"{start=}  {end=}  {vektorLen=:5.2f}m  {np.rad2deg(winkel_rad):.0f}°")
            if CheckWinkel(winkel_rad, ThetaWallSouth_rad) and vektorLen > 10.0:
                y = -DistY(start, end)
                print(f"Theta={theta_deg:.0f}° WallSouth detected {y:5.2f}m südlich vom Robotor")
            elif CheckWinkel(winkel_rad, ThetaWallNorth_rad) and vektorLen > 8.0:
                y = (start[1] + end[1]) / 2
                print(f"Theta={theta_deg:.0f}° WallNorth detected {y:5.2f}m nördlich vom Robotor")
            elif (CheckWinkel(winkel_rad, ThetaSchuppenEast_rad) and 
                  CheckLength(vektorLen, LenSchuppenEast) and
                  start[0] < 0 and end[0] < 0):
                x = (start[0] + end[0]) / 2
                print(f"Theta={theta_deg:.0f}° Schuppen detected ({vektorLen:5.2f}m breit) {x:5.2f}m westlich vom Robotor")


            #  ThetaWallNorth_rad     =  0.00000    #    0.0°
            #  ThetaWallEast_rad      =  1.48406    #   85.0°
            #  ThetaWallSouth_rad     = -0.36485    #  -20.9°
            #  ThetaWallWest_rad      =  1.45702    #   83.5°
            #  ThetaSchuppenEast_rad  =  np.pi/2    #   90.0°
            #  ThetaSchuppenSouth_rad =  0.00000    #    0.0°

    
    def StateMachine(self, scan_msg):
        now = self.get_clock().now().nanoseconds / 1e9
        all_detected_walls = self.Walldetector(scan_msg)
        PublishMarkers(self.marker_pub, all_detected_walls)
        self.counter += 1
        if self.state == STATE_INIT:     
            # Nach Norden ausrichten
            self.SetWantedTheta(np.pi/2)
            self.state = STATE_CHECK_NORTH
            
        elif self.state == STATE_CHECK_NORTH:
            if self.wantedThetaReached and now > self.wantedThetaReachedTime + 0.0:
                self.Localization(self.theta, all_detected_walls)
                # Nach Süden ausrichten
                self.SetWantedTheta(-np.pi/2)
                self.state = STATE_CHECK_SOUTH
                
        elif self.state == STATE_CHECK_SOUTH:
            if self.wantedThetaReached and now > self.wantedThetaReachedTime + 0.0:
                self.Localization(self.theta, all_detected_walls)
                self.state = STATE_INIT

def main():
    rclpy.init()
    node = Navigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Wird ausgelöst, wenn du Ctrl+C drückst
        node.get_logger().info("Simulation wird durch Benutzer abgebrochen...")
    except Exception as e:
        # Fängt unerwartete Fehler ab
        node.get_logger().error(f"Unerwarteter Fehler: {e}")
    finally:
        # Dieser Block wird IMMER ausgeführt, egal ob Fehler oder Ctrl+C
        node.get_logger().info("Bereinige Ressourcen...")
        # ROS nur herunterfahren, wenn es noch "ok" ist
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()