import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped, Twist
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
import socket
import struct
import math
import numpy as np # Vergiss nicht numpy zu importieren!
import sys
from sim import Simulation

sys.path.append('../HostSim')
import params
from params import LidarMaxAngle, LidarFreq_Hz


class SimNode(Node):
    def __init__(self, sim):
        super().__init__('simulator')

        self.sim=sim
        
        # Lidar scan parameters. One scan ranges from -LidarMaxAngle to +LidarMaxAngle
        self.scanDuration = LidarMaxAngle / 180.0 / LidarFreq_Hz
        self.scanTimeInc = self.scanDuration / (2*LidarMaxAngle)
        
        # ROS2 Parameter deklarieren (Name, Standardwert)
        self.declare_parameter('publish_odom_tf', True)
        
        self.publishOdomTf = self.get_parameter('publish_odom_tf').value
        self.get_logger().info(f"publish_odom_tf={self.publishOdomTf}")
        
        self.lidarCounter = 0
        
        clock_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.clock_pub = self.create_publisher(Clock, '/clock', clock_qos)

        # Custom Profil: Wir akzeptieren nur den EINEN neuesten Scan.
        # Alles was älter ist, wird sofort gelöscht.
        custom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Schnell, keine Garantien (wie UDP)
            history=HistoryPolicy.KEEP_LAST,
            depth=1,                                    # <--- WICHTIG: Puffergröße auf 1 zwingen!
            durability=DurabilityPolicy.VOLATILE)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', custom_qos)   #qos_profile_sensor_data)
        
        self.angle_pub = self.create_publisher(Float32, '/compass_heading', qos_profile_sensor_data)
        if self.publishOdomTf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscriber für Fahrbefehle von ROS 2
        self.cmd_sub = self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.cmd_vel_callback, 
            10)
        

        #self.create_timer(0.005, self.poll_udp)
        
        # Diese Meldung hat gefehlt:
        self.get_logger().info("Simulation running...")
        
        self.sim_time_sec = 0.0


    def cmd_vel_callback(self, msg):
        """Wird aufgerufen, wenn ROS2 einen Fahrbefehl sendet."""
        vLinear = msg.linear.x
        omega = msg.angular.z
        self.sim.SetRobotSpeed(vLinear, omega)


    def PublishLidarData(self, dist):
        # --- PUBLISH SCAN ---
        scan = LaserScan()
        # 2. Zur Sicherheit: time_increment nullen
        scan.time_increment = 0.0

        if False:
            now = self.get_clock().now()

            # Wir schieben den Scan 50ms in die Vergangenheit, damit der EKF Zeit hatte, 
            # die Position für diesen Moment bereits zu berechnen.
            scan.header.stamp = (now - rclpy.duration.Duration(seconds=0.2)).to_msg()

            # 3. Debug Print ZUR KONTROLLE (Was steht wirklich in der Nachricht?)
            #print(f"DEBUG: Bridge Time ist {now.nanoseconds/1e9}, aber Nachricht Header ist {scan.header.stamp.sec}.{scan.header.stamp.nanosec}")
        else:
            current_sim_time = Time(seconds=self.sim_time_sec)
            scan.header.stamp = current_sim_time.to_msg()
        scan.header.frame_id = 'lidar'      # wenn Lidar vor der Achsenmitte montiert ist (BackWheelDrive)
        #scan.header.frame_id = 'base_link'  # wenn Lidar direkt in Achsenmitte montiert ist
        #scan.time_increment = self.scanTimeInc
        scan.angle_min = math.radians(-LidarMaxAngle)
        scan.angle_max = math.radians(LidarMaxAngle-1)
        num_readings = 2*LidarMaxAngle
        scan.angle_increment = (scan.angle_max - scan.angle_min) / (num_readings - 1)
        scan.angle_max = scan.angle_min + (scan.angle_increment * (num_readings - 1))

        scan.range_min = params.LidarRangeMin
        scan.range_max = params.LidarRangeMax
        
        dist = np.clip(dist, scan.range_min, scan.range_max)
        scan.ranges = dist.tolist()

        #count = int(round((scan.angle_max - scan.angle_min) / scan.angle_increment))
        self.scan_pub.publish(scan)
                
    def PublishPositionAndTime(self, posX, posY, theta, simTimeSec):
        # Compute time
        current_time = Time(seconds=simTimeSec)
        tf_offset = Duration(seconds=0.1)
        tf_time = current_time   #+ tf_offset
        
        # --- PUBLISH CLOCK ---
        clock_msg = Clock()
        clock_msg.clock = current_time.to_msg()
        self.clock_pub.publish(clock_msg)

        # --- PUBLISH THETA ---
        msg = Float32()
        msg.data = float(theta)
        self.angle_pub.publish(msg)

        if self.publishOdomTf:
            # --- PUBLISH TF ---
            t = TransformStamped()
            t.header.stamp = tf_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = float(posX)
            t.transform.translation.y = float(posY)
            t.transform.translation.z = 0.0
            t.transform.rotation.z = math.sin(theta / 2.0)
            t.transform.rotation.w = math.cos(theta / 2.0)
            self.tf_broadcaster.sendTransform(t)

        # Kleine Erfolgsmeldung alle 100 Pakete
        theta_deg = int(np.rad2deg(theta))
        self.get_logger().info(
            f"[{simTimeSec:.3f}] Sende Position & Time #  {posX=:6.2f} {posY=:6.2f} {theta_deg}°", throttle_duration_sec=1.0)


import sys

def main():
    # ROS2 Initialisierung
    sim = Simulation()
    rclpy.init()
    node = SimNode(sim)

    try:
        # Die Schleife läuft nur, solange sim.running UND ROS okay ist
        while rclpy.ok() and sim.running:
            # A. ROS2 Events verarbeiten
            rclpy.spin_once(node, timeout_sec=0)

            # B. Simulations-Schritt
            x, y, theta, time, radius = sim.Step()
            
            # C. Nur publizieren, wenn wir nicht gerade herunterfahren
            if rclpy.ok() and not sim.pause:
                node.PublishPositionAndTime(x, y, theta, time)
                if len(radius) > 0: 
                    node.PublishLidarData(radius)
            
    except KeyboardInterrupt:
        # Wird ausgelöst, wenn du Ctrl+C drückst
        node.get_logger().info("Simulation wird durch Benutzer abgebrochen...")
    except Exception as e:
        # Fängt unerwartete Fehler ab
        node.get_logger().error(f"Unerwarteter Fehler: {e}")
    finally:
        # Dieser Block wird IMMER ausgeführt, egal ob Fehler oder Ctrl+C
        node.get_logger().info("Bereinige Ressourcen...")
        
        # Wichtig: Zuerst die Simulation (Fenster zu), dann ROS
        sim.Quit() 
        
        # Nur zerstören, wenn der Kontext noch gültig ist
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        
        # Optional: Komplettes Beenden erzwingen (hilft bei WSL2-Hängern)
        sys.exit(0)

    
if __name__ == '__main__':
    main()