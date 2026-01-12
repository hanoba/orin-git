import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from rclpy.qos import qos_profile_sensor_data
import numpy as np

class SmartCornerDetector(Node):
    def __init__(self):
        super().__init__('smart_corner_detector')
        
        # --- TUNING PARAMETER ---
        self.declare_parameter('distance_threshold', 0.05) 
        self.declare_parameter('min_points', 6)
        self.declare_parameter('min_length', 0.20)
        self.iterations = 40 

        self.dist_thresh = self.get_parameter('distance_threshold').value
        self.min_pts = self.get_parameter('min_points').value
        self.min_len = self.get_parameter('min_length').value

        # Subscription mit SensorData QoS (Best Effort)
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        
        # Publisher für Marker
        # --- STABILES QOS PROFIL ---
        # Wir nehmen RELIABLE, damit RViz die Marker nicht ignoriert
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        
        stable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        self.marker_pub = self.create_publisher(MarkerArray, '/garden_features', stable_qos)
        
        self.get_logger().info("✅ Detektor gestartet: Breite Linien & Tiefergelegt (Z=-0.1)")
        self.is_processing = False

    def polar_to_cartesian(self, ranges, angle_min, angle_inc):
        angles = angle_min + np.arange(len(ranges)) * angle_inc
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        return np.column_stack((x, y))

    def find_intersection(self, line1, line2):
        p1, p2 = line1; p3, p4 = line2
        x1, y1 = p1; x2, y2 = p2; x3, y3 = p3; x4, y4 = p4
        denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
        if denom == 0: return None 
        ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom
        return np.array([x1 + ua * (x2 - x1), y1 + ua * (y2 - y1)])

    def get_best_line_ransac(self, points):
        n_points = len(points)
        if n_points < self.min_pts: return None, None
        best_inliers_mask = None
        best_count = 0
        pts = points.astype(np.float32)

        for _ in range(self.iterations):
            idx = np.random.choice(n_points, 2, replace=False)
            p1, p2 = pts[idx[0]], pts[idx[1]]
            vec = p2 - p1
            norm = np.linalg.norm(vec)
            if norm < 0.01: continue 
            normal = np.array([-vec[1], vec[0]]) / norm
            dists = np.abs(np.dot(pts - p1, normal))
            current_mask = dists < self.dist_thresh
            count = np.sum(current_mask)
            if count > best_count:
                best_count = count
                best_inliers_mask = current_mask

        if best_count < self.min_pts: return None, None
        inliers = points[best_inliers_mask]
        mean = np.mean(inliers, axis=0)
        uu, dd, vv = np.linalg.svd(inliers - mean)
        direction = vv[0]
        projections = np.dot(inliers - mean, direction)
        p_start = mean + direction * np.min(projections)
        p_end = mean + direction * np.max(projections)
        if np.linalg.norm(p_end - p_start) < self.min_len: return None, None
        return (p_start, p_end), best_inliers_mask

    def scan_callback(self, msg):
        # Wenn wir noch rechnen, ignorieren wir den neuen Scan sofort
        if self.is_processing:
            return
            
        self.is_processing = True
        
        try:
            # Latenz Check gegen Lag
            now = self.get_clock().now()
            latency = (now - rclpy.time.Time.from_msg(msg.header.stamp)).nanoseconds / 1e9
            if latency > 0.3: return

            ranges = np.array(msg.ranges)
            valid = np.isfinite(ranges) & (ranges > 0.1) & (ranges < 30.0)
            if np.sum(valid) < self.min_pts: return
            points = self.polar_to_cartesian(ranges, msg.angle_min, msg.angle_increment)[valid]

            found_walls = []
            for _ in range(12): 
                if len(points) < self.min_pts: break
                line_geom, mask = self.get_best_line_ransac(points)
                if line_geom is None: break
                found_walls.append(line_geom)
                points = points[~mask]

            # MARKER ERZEUGEN
            markers = MarkerArray()
            
            # 1. WÄNDE (Breite grüne Balken unter dem Lidar)
            for i, (start, end) in enumerate(found_walls):
                m = Marker()
                m.header = msg.header
                m.ns = "walls"
                m.id = i
                m.type = Marker.LINE_STRIP
                m.action = Marker.ADD
                
                # DICKE UND POSITION
                m.scale.x = 0.15  # 15cm breit
                m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.8
                
                # LIFETIME (0.5s gegen Blinken)
                m.lifetime.nanosec = 500000000 
                
                # Z = -0.1 (Marker liegt 10cm unter den Lidar-Punkten)
                m.points = [
                    Point(x=start[0], y=start[1], z=-0.1), 
                    Point(x=end[0], y=end[1], z=-0.1)
                ]
                markers.markers.append(m)

            # 2. ECKEN (Gelbe Kugeln)
            corner_id = 100
            for i in range(len(found_walls)):
                for j in range(i + 1, len(found_walls)):
                    corner = self.find_intersection(found_walls[i], found_walls[j])
                    if corner is not None:
                        dists = [np.linalg.norm(corner - p) for p in [found_walls[i][0], found_walls[i][1], found_walls[j][0], found_walls[j][1]]]
                        if min(dists) < 1.5:
                            m = Marker()
                            m.header = msg.header
                            m.ns = "corners"
                            m.id = corner_id; corner_id += 1
                            m.type = Marker.SPHERE
                            m.lifetime.nanosec = 500000000
                            m.pose.position.x = float(corner[0])
                            m.pose.position.y = float(corner[1])
                            m.pose.position.z = -0.05 # Etwas höher als die Wand
                            m.scale.x = 0.25; m.scale.y = 0.25; m.scale.z = 0.25
                            m.color.r = 1.0; m.color.g = 0.9; m.color.b = 0.0; m.color.a = 1.0
                            markers.markers.append(m)

            self.marker_pub.publish(markers)
        finally:
            self.is_processing = False


def main():
    rclpy.init()
    node = SmartCornerDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()