#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class CompassFusionNode(Node):

    def __init__(self):
        super().__init__('compass_fusion_node')

        # --- EINSTELLUNGEN ---
        self.timer_rate = 20.0  # 20 Hz Output
        
        # Subscriber
        self.sub_laser = self.create_subscription(Odometry, '/rf2o_odom', self.laser_callback, 10)
        self.sub_compass = self.create_subscription(Float32, '/compass_heading', self.compass_callback, qos_profile_sensor_data)

        # Publisher & TF
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Interner Zustand
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.current_v = 0.0 
        self.heading_offset = None 
        self.last_input_theta = 0.0

        # --- GLITCH FILTER VARIABLEN ---
        self.last_valid_v = 0.0
        self.zero_vel_counter = 0
        # Wie viele "Null-Messungen" ignorieren wir? (bei 20Hz sind 10 ca. 0.5 Sekunden)
        self.MAX_ZERO_GLITCHES = 8 

        self.last_laser_time = self.get_clock().now()
        
        # Timer Loop
        self.timer = self.create_timer(1.0 / self.timer_rate, self.timer_callback)
        self.last_loop_time = self.get_clock().now()

        self.get_logger().info("🚀 Compass Fusion mit INERTIA-FILTER gestartet!")

    def compass_callback(self, msg):
        raw_angle = msg.data
        if self.heading_offset is None:
            self.heading_offset = raw_angle
            self.get_logger().info(f"🧭 Offset gesetzt: {self.heading_offset:.4f}")

        corrected = raw_angle - self.heading_offset
        # Wrap angle
        while corrected > math.pi: corrected -= 2 * math.pi
        while corrected < -math.pi: corrected += 2 * math.pi
        self.theta = corrected

    def laser_callback(self, msg):
        raw_v = msg.twist.twist.linear.x
        
        # --- DER TRÄGHEITS-FILTER ---
        # Wenn wir schnell waren (>0.1) und plötzlich 0.0 gemessen wird:
        if abs(self.last_valid_v) > 0.1 and abs(raw_v) < 0.001:
            self.zero_vel_counter += 1
            
            if self.zero_vel_counter <= self.MAX_ZERO_GLITCHES:
                # Wir ignorieren die Null und nutzen den alten Wert weiter!
                # Wir tun so, als würde er weiterrollen (was er ja tut)
                self.current_v = self.last_valid_v
                # Optional: Kleines Decay (langsames Ausrollen simulieren)
                self.current_v *= 0.95 
            else:
                # Okay, es ist keine Störung, er steht wirklich.
                self.current_v = 0.0
                self.last_valid_v = 0.0
        else:
            # Alles normal (normale Beschleunigung/Bremsung)
            self.zero_vel_counter = 0
            self.current_v = raw_v
            self.last_valid_v = raw_v

        self.last_laser_time = self.get_clock().now()

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_loop_time).nanoseconds / 1e9
        self.last_loop_time = now
        if dt <= 0: return
        
        # Timeout Check (falls rf2o ganz abstürzt)
        time_since_laser = (now - self.last_laser_time).nanoseconds / 1e9
        if time_since_laser > 0.5:
            v_to_use = 0.0 # Safety Stop
        else:
            v_to_use = self.current_v

        # Winkel-Delta (vom Kompass)
        delta_theta_raw = self.theta - self.last_input_theta
        delta_theta = math.atan2(math.sin(delta_theta_raw), math.cos(delta_theta_raw))
        
        angular_velocity = delta_theta / dt
        self.last_input_theta = self.theta

        # Position berechnen
        avg_theta = self.theta - (delta_theta / 2.0)
        delta_x = (v_to_use * math.cos(avg_theta)) * dt
        delta_y = (v_to_use * math.sin(avg_theta)) * dt

        self.x += delta_x
        self.y += delta_y

        self.publish_odometry(v_to_use, angular_velocity, now)

    def publish_odometry(self, v, omega, time_stamp):
        odom = Odometry()
        odom.header.stamp = time_stamp.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega
        
        self.pub_odom.publish(odom)

        t = TransformStamped()
        t.header.stamp = time_stamp.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = CompassFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    