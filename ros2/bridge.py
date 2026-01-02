import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
import socket
import struct
import math
import numpy as np # Vergiss nicht numpy zu importieren!
import sys

sys.path.append('../HostSim')
from Lidar import LidarMaxAngle


class IsaacBridge(Node):
    def __init__(self):
        super().__init__('meine_udp_bridge')
        
        #qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        clock_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
)        
        self.clock_pub = self.create_publisher(Clock, '/clock', clock_qos)

        #self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscriber für Fahrbefehle von ROS 2
        self.cmd_sub = self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.cmd_vel_callback, 
            10)
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind(("127.0.0.1", 5005))
        except Exception as e:
            print(f"Fehler beim Binden des Sockets: {e}")
            sys.exit(1)
        self.sock.setblocking(False)

        # Zieladresse für Rückrichtung (Isaac Sim auf Windows)
        self.isaac_addr = ("127.0.0.1", 5006)        

        self.create_timer(0.005, self.poll_udp)
        
        # Diese Meldung hat gefehlt:
        self.get_logger().info("Bidirektionale Bridge läuft auf Ports 5005 und 5006. Warte auf Isaac Sim...")

    def cmd_vel_callback(self, msg):
        """Wird aufgerufen, wenn ROS 2 einen Fahrbefehl sendet."""
        vLinear = msg.linear.x
        omega = msg.angular.z

        cmd = np.array([1, vLinear, omega], dtype=float)
        packet = cmd.tobytes()
        
        try:
            self.sock.sendto(packet, self.isaac_addr)
        except Exception as e:
            self.get_logger().error(f"Fehler beim Senden an Isaac: {e}")

    def poll_udp(self):
        try:
            data, addr = self.sock.recvfrom(2048)
        except BlockingIOError:
            return
        except Exception as e:
            self.get_logger().error(f"UDP Recv Error: {e}")
            return

        try:
            # 1. Daten zerlegen
            lidar_bytes = data[:720]
            # Sicherstellen, dass wir uint16 lesen (2 Byte pro Wert)
            radius = np.frombuffer(lidar_bytes, dtype=np.uint16)
            
            # Umsortieren von 0° bis 360° auf -120° bis 120° = -LidarMaxAngle bis LidarMaxAngle 
            # (ROS2 kann 360 Werte nicht verarbeiten, daher reduzierter Winkelbereich)
            dist_mm = np.concatenate((radius[360-LidarMaxAngle:360], radius[0:LidarMaxAngle]))
            
            # WICHTIG: Explizit nach float32 konvertieren für ROS!
            dist = dist_mm.astype(np.float32) / 1000.0

            # 2. Odometrie entpacken
            odometry_time_bytes = data[720:]
            # < = Little Endian, fff = 3x Float, d = 1x Double
            posX, posY, theta_deg, sim_time_sec = struct.unpack('<fffd', odometry_time_bytes)
            theta = math.radians(theta_deg)
            
            # Compute time
            #seconds = int(sim_time_sec)
            #nanoseconds = int((sim_time_sec - seconds) * 1e9)
            current_time = Time(seconds=sim_time_sec)
            tf_offset = Duration(seconds=0.1)
            tf_time = current_time + tf_offset
            timestamp = current_time.to_msg()
            
            # --- PUBLISH CLOCK ---
            clock_msg = Clock()
            clock_msg.clock = timestamp
            #clock_msg.clock.sec = seconds
            #clock_msg.clock.nanosec = nanoseconds
            self.clock_pub.publish(clock_msg)
            
            #timestamp = Time(seconds=seconds, nanoseconds=nanoseconds).to_msg()
            #timestamp = self.get_clock().now().to_msg()

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

            # --- PUBLISH SCAN ---
            scan = LaserScan()
            scan.header.stamp = timestamp
            #scan.header.frame_id = 'lidar'  # 'base_link'
            scan.header.frame_id = 'base_link'
            LidarMaxAngleRad = math.radians(LidarMaxAngle)
            scan.angle_min = -LidarMaxAngleRad
            scan.angle_max =  LidarMaxAngleRad
            num_readings = 2*LidarMaxAngle
            scan.angle_increment = (scan.angle_max - scan.angle_min) / num_readings
            scan.angle_max = scan.angle_min + (scan.angle_increment * (num_readings - 1))

            scan.range_min = 0.1
            scan.range_max = 80.0
            
            dist = np.clip(dist, scan.range_min, scan.range_max)
            scan.ranges = dist.tolist()

            count = int(round((scan.angle_max - scan.angle_min) / scan.angle_increment))
            self.scan_pub.publish(scan)
            
            # Kleine Erfolgsmeldung alle 10 Pakete
            if not hasattr(self, 'count'): self.count = 0
            self.count += 1
            if self.count % 10 == 0:
                self.get_logger().info(f"Sende Scan #{self.count} bei SimTime {sim_time_sec:.2f}  {count=}  {posX=:6.2f} {posY=:6.2f} {int(theta_deg)}°")

        except Exception as e:
            # DAS hier wird uns den Fehler im Terminal anzeigen!
            self.get_logger().error(f"Fehler bei der Datenverarbeitung: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = IsaacBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()