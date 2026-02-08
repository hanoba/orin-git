import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32
import numpy as np
import time
import sys
import math
from Ransac import LineDetection, PublishMarkers
from GartenWorld import Localization, lineNames
from TaskLists import LocalizationTaskList, FahreInDenWaldTaskList

sys.path.append('../HostSim')
import params


def NormalizeAngle(angle_rad):
    return (angle_rad + math.pi) % math.tau - np.pi

def CheckAngle(angle_rad, value_rad):
    MaxDiff = np.deg2rad(5.0)
    diff1 = NormalizeAngle(angle_rad - value_rad)
    diff2 = NormalizeAngle(angle_rad + math.pi - value_rad)
    return abs(diff1) < MaxDiff or abs(diff2) < MaxDiff 

def CheckAngleResult_deg(angle_rad, value_rad):
    diff1 = NormalizeAngle(angle_rad - value_rad)
    diff2 = NormalizeAngle(angle_rad + math.pi - value_rad)
    return np.rad2deg(min(abs(diff1), abs(diff2)))

def CheckLength(vektorLen, value):
    Margin = 0.50
    if vektorLen < value - Margin: return False
    if vektorLen > value + Margin: return False
    return True

def LineEquation(A, B):
    """ Berechne die Parameter a und b der Geraden y=a*x+b, die durch die Punkte A und B geht """
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
        # NEU: Wenn Punkte weiter als max_gap auseinander liegen -> Tor/Lücke
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
        self.angularMax = 2.0*2
        self.angular = 0.0
        self.linear = 0.0
        self.directionFlag = False
        self.simTimeSec = 0.0
                
        # Publisher für die Fahrbefehle
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Publisher für rviz Textausgabe:
        self.text_pub = self.create_publisher(Marker, 'text_marker_topic', 10)
        
        # Set initial task list
        self.NewTaskList(FahreInDenWaldTaskList)

    def SetVelocities(self, omega, vLinear):
        self.angular = omega
        self.linear = vLinear
        self.wantedThetaReached = True
        self.directionFlag = False
        
    def SetWantedTheta(self, wantedTheta):
        self.wantedTheta = wantedTheta
        self.wantedThetaReached = False
        self.directionFlag = False
        self.get_logger().info(f"[{self.simTimeSec:.3f}] [SetWantedTheta] wantedTheta={self.wantedTheta}")
        self.linear = 0.0

    def SetDirection(self, theta, vLinear):
        self.directionFlag = True
        self.wantedTheta = theta
        self.linear = vLinear
        self.wantedThetaReached = True

    def ResetDirection(self):
        self.linear = 0.0
        self.angular = 0.0
        self.wantedThetaReached = True
        self.directionFlag = False

    def CompassCallback(self, msg):
        self.theta = NormalizeAngle(msg.data)
        if self.directionFlag:
            e = self.wantedTheta - self.theta
            e = (e + math.pi) % math.tau - np.pi
            e = np.clip(e, -self.angularMax, self.angularMax)
            self.angular = e * self.K_head
        elif not self.wantedThetaReached:
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
        #self.StateMachine(msg)
        self.TaskStep(msg)
        self.is_processing = False

    def Walldetector(self, msg):
        start_zeit = time.perf_counter() # Zeitnahme startet
        ranges = np.array(msg.ranges)
        valid = np.isfinite(ranges) & (ranges > params.LidarRangeMin + 0.01) & (ranges < params.LidarRangeMax - 0.1)
        #valid = np.isfinite(ranges) & (ranges > 0.1) & (ranges < 30.0)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        
        points = np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))[valid]

        all_detected_walls = LineDetection(points)
        
        ende_zeit = time.perf_counter() # Zeitnahme endet
        dauer_ms = (ende_zeit - start_zeit) * 1000 # Umrechnung in Millisekunden
        
        # Ausgabe im Terminal (alle Sekunde, um das Terminal nicht zu fluten)
        self.get_logger().info(f"[{self.simTimeSec:.3f}] [Walldetector] ⏱️ Rechenzeit: {dauer_ms:.2f} ms  Theta={np.rad2deg(self.theta):.0f}°", throttle_duration_sec=100.0)
        return all_detected_walls

    def NewTaskList(self, taskList):
        self.taskList = taskList
        self.taskIndex = 0
        if self.taskIndex < len(self.taskList):
            task, params = self.taskList[self.taskIndex]
            task.Init(self, params)
        
    def TaskStep(self, scan_msg):
        if self.taskIndex < len(self.taskList):
            task, params = self.taskList[self.taskIndex]
            self.retvals = task.Step(scan_msg)
            if self.retvals is not None:
                nextTaskIndex = self.taskIndex+1
                if nextTaskIndex < len(self.taskList):
                    self.GotoTask(nextTaskIndex)

    def GotoTask(self, taskIndex):
        self.taskIndex = taskIndex
        if self.taskIndex < len(self.taskList):
            task, params = self.taskList[self.taskIndex]
            task.Init(self, params, self.retvals)
        else:
            self.get_logger().error(f"[GotoTask] Illegal task index: {taskIndex}")

    def RvizPrint(self, text):
        marker = Marker()
        marker.header.frame_id = "map"  # Muss mit deinem Fixed Frame in RViz übereinstimmen
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Inhalt des Strings
        marker.text = text
        self.get_logger().info(text)
        
        # Position und Größe
        # Position setzen
        marker.pose.position.x = 0.0  # 2 Meter nach vorne
        marker.pose.position.y = 13.0 # 1,5 Meter nach rechts
        marker.pose.position.z = 0.5  # 0,5 Meter über dem Boden
        marker.pose.position.z = 1.0  # Schwebt 1 Meter über dem Boden
        marker.scale.z = 1.5          # Texthöhe in Metern
        
        # Farbe (RGBA) - Weiß und voll sichtbar
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        self.text_pub.publish(marker)    
def main():
    rclpy.init()
    node = Navigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Wird ausgelöst, wenn du Ctrl+C drückst
        node.get_logger().info("Simulation wird durch Benutzer abgebrochen...")
    #except Exception as e:
    #    # Fängt unerwartete Fehler ab
    #    node.get_logger().error(f"Unerwarteter Fehler: {e}")
    finally:
        # Dieser Block wird IMMER ausgeführt, egal ob Fehler oder Ctrl+C
        node.get_logger().info("Bereinige Ressourcen...")
        # ROS nur herunterfahren, wenn es noch "ok" ist
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()