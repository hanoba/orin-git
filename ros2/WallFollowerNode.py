#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import sys

sys.path.append('../HostSim')
from WallFollower import WallFollower

class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower')
        
        # 1. Subscriber für die Lidar-Daten
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        
        # 2. Publisher für die Fahrbefehle
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info("Wall Follower gestartet...")
        
        self.wallFollower = WallFollower(target_dist=2.00, base_speed=0.5, debug=False)
        self.cnt = 0

    def scan_callback(self, msg):
        self.cnt += 1
        if self.cnt % 50 == 0: self.get_logger().info(f"scan_callback #{self.cnt}")
        ranges_np = np.array(msg.ranges)
        vLinear, omega = self.wallFollower.step(ranges_np)    
        drive_msg = Twist()
        drive_msg.linear.x = float(vLinear)
        drive_msg.angular.z = float(omega)
        self.cmd_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()