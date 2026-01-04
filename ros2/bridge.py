import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
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
        super().__init__('isaac_bridge')
        
        #qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        clock_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
            
        self.clock_pub = self.create_publisher(Clock, '/clock', clock_qos)

        # Definiere ein robustes QoS-Profil
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, # Das löst die Warnung aus der Welt
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        #self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        #self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile)
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
        
        self.sim_time_sec = 0.0


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
            dataLen = len(data)
            if dataLen == 720:
                # Sicherstellen, dass wir uint16 lesen (2 Byte pro Wert)
                radius = np.frombuffer(data, dtype=np.uint16)
                
                # Umsortieren von 0° bis 360° auf -120° bis 120° = -LidarMaxAngle bis LidarMaxAngle 
                # (ROS2 kann 360 Werte nicht verarbeiten, daher reduzierter Winkelbereich)
                dist_mm = np.concatenate((radius[360-LidarMaxAngle:360], radius[0:LidarMaxAngle]))
                
                # WICHTIG: Explizit nach float32 konvertieren für ROS!
                dist = dist_mm.astype(np.float32) / 1000.0

                # --- PUBLISH SCAN ---
                current_time = Time(seconds=self.sim_time_sec)
                scan = LaserScan()
                scan.header.stamp = current_time.to_msg()
                scan.header.frame_id = 'lidar'      # wenn Lidar vor der Achsenmitte montiert ist (BackWheelDrive)
                #scan.header.frame_id = 'base_link'  # wenn Lidar direkt in Achsenmitte montiert ist
                LidarMaxAngleRad = math.radians(LidarMaxAngle)
                scan.angle_min = -LidarMaxAngleRad
                scan.angle_max =  LidarMaxAngleRad
                num_readings = 2*LidarMaxAngle
                scan.angle_increment = (scan.angle_max - scan.angle_min) / (num_readings - 1)
                scan.angle_max = scan.angle_min + (scan.angle_increment * (num_readings - 1))

                scan.range_min = 0.1
                scan.range_max = 80.0
                
                dist = np.clip(dist, scan.range_min, scan.range_max)
                scan.ranges = dist.tolist()

                #count = int(round((scan.angle_max - scan.angle_min) / scan.angle_increment))
                self.scan_pub.publish(scan)
                
            elif dataLen == 20:
                # 2. Odometrie entpacken
                # < = Little Endian, fff = 3x Float, d = 1x Double
                posX, posY, theta_deg, self.sim_time_sec = struct.unpack('<fffd', data)
                theta = math.radians(theta_deg)
                
                # Compute time
                current_time = Time(seconds=self.sim_time_sec)
                tf_offset = Duration(seconds=0.1)
                tf_time = current_time   #+ tf_offset
                
                # --- PUBLISH CLOCK ---
                clock_msg = Clock()
                clock_msg.clock = current_time.to_msg()
                self.clock_pub.publish(clock_msg)
                
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
                if not hasattr(self, 'count'): self.count = 0
                self.count += 1
                if self.count % 100 == 0:
                    self.get_logger().info(
                        f"Sende Position & Time #{self.count} bei SimTime {self.sim_time_sec:.2f}  {posX=:6.2f} {posY=:6.2f} {int(theta_deg)}°")
            else:
                self.get_logger().error(f"Wrong UDP data length: {dataLen} bytes")            
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