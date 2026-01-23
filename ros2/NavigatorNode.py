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

def NormalizeAngle(angle_rad):
    return (angle_rad + math.pi) % math.tau - np.pi

def CheckAngle(angle_rad, value_rad):
    MaxDiff = np.deg2rad(5.0)
    diff1 = NormalizeAngle(angle_rad - value_rad)
    diff2 = NormalizeAngle(angle_rad + math.pi - value_rad)
    return abs(diff1) < MaxDiff or abs(diff2) < MaxDiff 

def CheckAngleResult_deg(angle_rad, value_rad):
    MaxDiff = np.deg2rad(5.0)
    diff1 = NormalizeAngle(angle_rad - value_rad)
    diff2 = NormalizeAngle(angle_rad + math.pi - value_rad)
    return np.rad2deg(min(abs(diff1), abs(diff2)))

def CheckLength(vektorLen, value):
    Margin = 0.50
    if vektorLen < value - Margin: return False
    if vektorLen > value + Margin: return False
    return True

def LineEquation(A, B):
    # Berechne die Parameter a und b der Geraden y=a*x+b, die durch die Punkte A und B geht
    a = (B[1] - A[1]) / (B[0] - A[0])
    b = A[1] - a*A[0]
    return a, b

def DistY(start, end):
    """ Berechne den Abstand in Y-Richtung von der Geraden, die durch die Punkte start und end geht """
    a, b = LineEquation(start, end)
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
        self.wantedThetaReached = True
        self.K_head = 1.0
        self.angularMax = 2.0
        self.angular = 0.0
        self.linear = 0.0
        self.state = STATE_INIT
        self.simTimeSec = 0.0
                
        # Publisher für die Fahrbefehle
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
    def SetWantedTheta(self, wantedTheta):
        self.wantedTheta = wantedTheta
        self.wantedThetaReached = False
        self.get_logger().info(f"[{self.simTimeSec:.3f}] [SetWantedTheta] wantedTheta={self.wantedTheta}")

    def CompassCallback(self, msg):
        self.theta = NormalizeAngle(msg.data)
        if not self.wantedThetaReached:
            e = self.wantedTheta - self.theta
            e = (e + math.pi) % math.tau - np.pi
            e = np.clip(e, -self.angularMax, self.angularMax)
            #print(f"{self.theta=}  {e=}  {self.wantedTheta=}")        
            if abs(e) < 0.002: 
                self.wantedThetaReachedTime = self.get_clock().now().nanoseconds / 1e9
                self.wantedThetaReached = True
                self.get_logger().info(f"[{self.wantedThetaReachedTime:.3f}] [CompassCallback] wantedThetaReached")
                self.angular = 0.0
            else: 
                self.angular = e * self.K_head
        drive_msg = Twist()
        drive_msg.linear.x = float(self.linear)
        drive_msg.angular.z = float(self.angular)
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
        self.get_logger().info(f"[{self.simTimeSec:.3f}] [Walldetector] ⏱️ Rechenzeit: {dauer_ms:.2f} ms  Theta={np.rad2deg(self.theta):.0f}°", throttle_duration_sec=1.0)
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
        # self.get_logger().info(f"[{self.simTimeSec:.3f}] [Localization] all_detected_walls: {a.shape=}  {a.dtype=} {a.ndim=} {a.size=}",
        #     throttle_duration_sec=1.0)
        for start, end in a:   # all_detected_walls:
            # Drehung durch Matrix-Multiplikation (Dot Product)
            # Wir transponieren die Matrix (.T), damit (N,2) * (2,2) funktioniert        
            start = np.dot(start, rotation_matrix.T)
            end = np.dot(end, rotation_matrix.T)
            vektor = end - start
            vektorLen = np.linalg.norm(vektor)
            angle_rad = np.arctan2(vektor[1], vektor[0])
            #self.get_logger().info(f"[{self.simTimeSec:.3f}] [Localization] {start=}  {end=}  {vektorLen=:5.2f}m  {np.rad2deg(angle_rad):.0f}°")
            if CheckAngle(angle_rad, ThetaWallSouth_rad) and vektorLen > 10.0:
                y = -DistY(start, end)
                self.get_logger().info(f"[{self.simTimeSec:.3f}] [Localization] Theta={theta_deg:.0f}° WallSouth erkannt {y:5.2f}m südlich vom Robotor")
            if CheckAngle(angle_rad, ThetaWallNorth_rad) and vektorLen > 8.0:
                y = (start[1] + end[1]) / 2
                self.get_logger().info(f"[{self.simTimeSec:.3f}] [Localization] Theta={theta_deg:.0f}° WallNorth erkannt {y:5.2f}m nördlich vom Robotor")
            if (CheckAngle(angle_rad, ThetaSchuppenEast_rad) and 
                CheckLength(vektorLen, LenSchuppenEast) and
                start[0] < 0 and end[0] < 0
            ):
                x = (start[0] + end[0]) / 2
                self.get_logger().info( 
                    f"[{self.simTimeSec:.3f}] [Localization] Theta={theta_deg:.0f}° "
                    f"Schuppen erkannt ({vektorLen:.2f}m breit) {x:5.2f}m westlich vom Robotor")                    
            #self.get_logger().info(f"{CheckAngleResult_deg(angle_rad, ThetaSchuppenSouth_rad)=:.1f}° {np.rad2deg(angle_rad)=:.1f}°")      
            if (CheckAngle(angle_rad, ThetaSchuppenSouth_rad) and 
                CheckLength(vektorLen, LenSchuppenSouth) and
                start[1] > 0 and end[1] > 0
            ):
                y = DistY(start, end)
                self.get_logger().info( 
                    f"[{self.simTimeSec:.3f}] [Localization] Theta={theta_deg:.0f}° "
                    f"Schuppen erkannt ({vektorLen:.2f}m breit) {y:5.2f}m nördlich vom Robotor")


            #  ThetaWallNorth_rad     =  0.00000    #    0.0°
            #  ThetaWallEast_rad      =  1.48406    #   85.0°
            #  ThetaWallSouth_rad     = -0.36485    #  -20.9°
            #  ThetaWallWest_rad      =  1.45702    #   83.5°
            #  ThetaSchuppenEast_rad  =  np.pi/2    #   90.0°
            #  ThetaSchuppenSouth_rad =  0.00000    #    0.0°

    
    def StateMachine(self, scan_msg):
        self.simTimeSec = self.get_clock().now().nanoseconds / 1e9
        all_detected_walls = self.Walldetector(scan_msg)
        PublishMarkers(self.marker_pub, all_detected_walls)
        if self.state == STATE_INIT:     
            # Nach Norden ausrichten
            self.SetWantedTheta(np.pi/2)
            self.state = STATE_CHECK_NORTH
            
        elif self.state == STATE_CHECK_NORTH:
            if self.wantedThetaReached and self.simTimeSec > self.wantedThetaReachedTime + 0.5:
                self.Localization(self.theta, all_detected_walls)
                # Nach Süden ausrichten
                self.SetWantedTheta(-np.pi/2)
                self.state = STATE_CHECK_SOUTH
                
        elif self.state == STATE_CHECK_SOUTH:
            if self.wantedThetaReached and self.simTimeSec > self.wantedThetaReachedTime + 0.5:
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