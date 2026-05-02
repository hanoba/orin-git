import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
import numpy as np
import time
import math
import Ransac
import TaskLists

from params import TaskState


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

# Odometrie-Funktionen zur Schätzung der Position
class Odometry:   
    def __init__(self):
        self.pos = np.array([0.0, 0.0])    
        self.theta = 0
    
    def SetPos(self, x, theta):
        """ Wird immer aufgerufen, wenn Position bestimmt wurde """
        self.pos = x
        self.theta = theta
       
    def GetPos(self):
        return float(self.pos[0]), float(self.pos[1])
       
    def SetStartPoint(self, dist, theta):
        """ Wird von FollowPathTask aufgerufen, wenn neues Pfadsegment Format 1 gestartet wird. """
        self.theta = theta
        self.startDist = dist
        self.startPos = self.pos
        self.heading = np.array([math.cos(theta), math.sin(theta)])
        
    def UpdatePos(self, dist):
        d = self.startDist - dist
        self.pos = self.startPos + d*self.heading


class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self.declare_parameter('distance_threshold', 0.05) 
        self.declare_parameter('min_points', 6)
        # NEU: Wenn Punkte weiter als max_gap auseinander liegen -> Tor/Lücke

        self.declare_parameter('max_gap', 5.50) 
        # ROS2 Parameter deklarieren (Name, Standardwert)
        self.declare_parameter('publish_odom_tf', False)
        
        self.publishOdomTf = self.get_parameter('publish_odom_tf').value
        self.get_logger().info(f"publish_odom_tf={self.publishOdomTf}")

        # Odometrie-Funktionen
        self.odom = Odometry()

    
        if self.publishOdomTf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
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
            qos_policy
            #qos_profile_sensor_data
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
        
        self.retvals = None
                
        # Publisher für die Fahrbefehle
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos_policy)
        
        # Publisher für rviz Textausgabe:
        self.text_pub = self.create_publisher(Marker, 'text_marker_topic', 10)

        # Services erstellen: Typ (Trigger)
        self.srv1 = self.create_service(Trigger, 'Localization',                    self.cb_Localization)
        self.srv2 = self.create_service(Trigger, 'Mowing',                          self.cb_Mowing)
        self.srv3 = self.create_service(Trigger, 'Fahre_zum_Schuppen',              self.cb_Fahre_zum_Schuppen)
        self.srv4 = self.create_service(Trigger, 'Fahre_in_den_Wald',               self.cb_Fahre_in_den_Wald)
        self.srv5 = self.create_service(Trigger, 'Fahre_in_den_Garten',             self.cb_Fahre_in_den_Garten)
        self.srv6 = self.create_service(Trigger, 'Fahre_hinters_Haus',              self.cb_Fahre_hinters_Haus)
        self.srv7 = self.create_service(Trigger, 'Stop',                            self.cb_Stop)
        self.srv8 = self.create_service(Trigger, 'Bestimme_YawOffset',              self.cb_Bestimme_YawOffset)
        self.srv9 = self.create_service(Trigger, 'Test',                            self.cb_Test)
        
        
        # Set empty task list
        self.Reset()
        self.missedScans = 0
    
    def SetVelocities(self, omega, vLinear):
        self.angular = 0.0
        self.linear = 0.0
        self.wantedThetaReached = True
        self.directionFlag = False
        self.PubVelocities(omega, vLinear)
        
    def PubVelocities(self, omega, linear):
        drive_msg = Twist()
        drive_msg.linear.x = float(linear)
        drive_msg.angular.z = float(omega)
        self.cmd_pub.publish(drive_msg)
        
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
        self.SetVelocities(0.0, 0.0)

    def CompassCallback(self, msg):
        self.theta = NormalizeAngle(msg.data) 
        # self.theta += np.radians(10.0)    # for test of YawOffsetDetectionTask.py
        if self.directionFlag:
            e = self.wantedTheta - self.theta
            e = (e + math.pi) % math.tau - np.pi
            e = np.clip(e, -self.angularMax, self.angularMax)
            self.angular = e * self.K_head
            self.PubVelocities(self.angular, self.linear)    
        elif not self.wantedThetaReached:
            e = self.wantedTheta - self.theta
            e = (e + math.pi) % math.tau - np.pi
            e = np.clip(e, -self.angularMax, self.angularMax)
            #print(f"{self.theta=}  {e=}  {self.wantedTheta=}")        
            if abs(e) < np.deg2rad(10): # 0.002: 
                self.wantedThetaReachedTime = self.get_clock().now().nanoseconds / 1e9
                self.wantedThetaReached = True
                self.get_logger().info(f"[{self.wantedThetaReachedTime:.3f}] [CompassCallback] wantedThetaReached")
                self.angular = 0.0
            else: 
                self.angular = e * self.K_head
            self.PubVelocities(self.angular, self.linear)    

    def ScanCallback(self, scan_msg):
        if self.is_processing: 
            self.missedScans += 1
            return
        start_zeit = time.perf_counter() # Zeitnahme startet
        self.is_processing = True
        # Lidardaten umdrehen bei FrontWheelDrive
        #if self.frontWheelDrive:
        #    scan_msg.ranges = scan_msg.ranges[::-1]
        #    angle_min = scan_msg.angle_min
        #    scan_msg.angle_min = -scan_msg.angle_max
        #    scan_msg.angle_max = -angle_min
        self.TaskStep(scan_msg)
        self.is_processing = False
        ende_zeit = time.perf_counter() # Zeitnahme endet
        dauer_ms = (ende_zeit - start_zeit) * 1000 # Umrechnung in Millisekunden

        # Publish estimated position
        if self.publishOdomTf:
            t = TransformStamped()
            # WICHTIG: Auf Hardware immer die aktuelle Systemzeit nutzen!
            t.header.stamp = self.get_clock().now().to_msg() 

            t.header.frame_id = 'map'       # Die Karte ist der Ursprung
            t.child_frame_id = 'odom'       # Wir bewegen den Odom-Frame (der starr am Roboter klebt)

            # Deine geschätzte Position (Absolute Koordinaten auf der Karte)
            posX, posY = self.odom.GetPos()
            
            t.transform.translation.x = float(posX)
            t.transform.translation.y = float(posY)
            t.transform.translation.z = 0.0

            # Rotation aus deinem geschätzten Theta
            t.transform.rotation.z = math.sin(self.theta / 2.0)
            t.transform.rotation.w = math.cos(self.theta / 2.0)

            self.tf_broadcaster.sendTransform(t)
        
        # Ausgabe im Terminal (alle Sekunde, um das Terminal nicht zu fluten)
        self.get_logger().info(
            f"[ScanCallback] ⏱️ Rechenzeit: {dauer_ms:.2f} ms  "
            f"Missed Scans: {self.missedScans}  Theta={np.rad2deg(self.theta):.0f}°", throttle_duration_sec=10.0)

    def Walldetector(self, msg, angMin=-np.pi, angMax=np.pi):
        ranges = np.array(msg.ranges)
        #rangeMask = np.isfinite(ranges) & (ranges > params.LidarRangeMin + 0.01) & (ranges < params.LidarRangeMax - 0.1)
        rangeMask = np.isfinite(ranges) & (ranges > msg.range_min + 0.01) & (ranges < msg.range_max - 0.1)
        #valid = np.isfinite(ranges) & (ranges > 0.1) & (ranges < 30.0)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        angMask = (angles >=angMin) & (angles <= angMax)
        valid = rangeMask & angMask
        
        points = np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))[valid]

        all_detected_walls = Ransac.LineDetection(points)
        
        return all_detected_walls

    # Callback functions for trigger services
    def cb_Localization(self, req, resp):          return self.Service(req, resp, TaskLists.Localization_TaskList)
    def cb_Mowing(self, req, resp):                return self.Service(req, resp, TaskLists.Mowing_TaskList)
    def cb_Fahre_zum_Schuppen(self, req, resp):    return self.Service(req, resp, TaskLists.Fahre_zum_Schuppen_TaskList)
    def cb_Fahre_in_den_Wald(self, req, resp):     return self.Service(req, resp, TaskLists.Fahre_in_den_Wald_TaskList)
    def cb_Fahre_in_den_Garten(self, req, resp):   return self.Service(req, resp, TaskLists.Fahre_in_den_Garten_TaskList)
    def cb_Fahre_hinters_Haus(self, req, resp):    return self.Service(req, resp, TaskLists.Fahre_hinters_Haus_TaskList)
    def cb_Bestimme_YawOffset(self, req, resp):    return self.Service(req, resp, TaskLists.Bestimme_YawOffset_TaskList)
    def cb_Test(self, req, resp):                  return self.Service(req, resp, TaskLists.Test_TaskList)
    
    def cb_Stop(self, request, response):
        self.Reset()
        self.RvizPrint("Reset")
        response.success = True 
        response.message = "Ein Reset wurde durchgeführt"
        return response

    def Service(self, request, response, taskList):
        # 1. Das Signal ist angekommen! Hier führst du deine Aktion aus.
        taskListName = taskList["name"]
        self.get_logger().info(f"Trigger empfangen für {taskListName}")
        self.NewTaskList(taskList)
        response.success = True 
        response.message = f"Die Taskliste {taskListName} wurde gestartet"
        return response
        
    def Reset(self):
        self.taskListName = "None"
        self.taskList = None
        self.taskIndex = 0
        self.SetVelocities(0.0, 0.0)
        print("Reset() called")
        
    def NewTaskList(self, taskList):
        self.taskListName = taskList["name"]
        self.taskList = taskList["tasks"]
        self.taskIndex = 0
        if self.taskIndex < len(self.taskList):
            task, params = self.taskList[self.taskIndex]
            task.Init(self, params, self.retvals)
        
    def TaskStep(self, scan_msg):
        if self.taskList is not None:
            if self.taskIndex >= len(self.taskList):
                self.Reset()
            else:    
                task, params = self.taskList[self.taskIndex]
                status, self.retvals = task.Step(scan_msg)
                if status == TaskState.Ready:
                    nextTaskIndex = self.taskIndex+1
                    if nextTaskIndex < len(self.taskList):
                        self.GotoTask(nextTaskIndex)
                    else:
                        self.Reset()
                elif status == TaskState.Error:
                    self.get_logger().error(f"[TaskStep] Error in task with task index: {self.taskIndex}")
                    self.Reset()

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
        marker.text = f"{self.taskListName}: {text}"
        self.get_logger().info(text)
        
        # Position und Größe
        # Position setzen
        marker.pose.position.x = 0.0  # 2 Meter nach vorne
        marker.pose.position.y = -14.0 # 1,5 Meter nach rechts
        marker.pose.position.z = 0.5  # 0,5 Meter über dem Boden
        #marker.pose.position.z = 1.0  # Schwebt 1 Meter über dem Boden
        marker.scale.z = 1.0          # Texthöhe in Metern
        
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