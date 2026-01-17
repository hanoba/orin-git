import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import time

class SmartCornerDetector(Node):
    def __init__(self):
        super().__init__('smart_corner_detector')
        self.declare_parameter('distance_threshold', 0.05) 
        self.declare_parameter('min_points', 6)
        # NEU: Wenn Punkte weiter als 50cm auseinander liegen -> Tor/Lücke
        self.declare_parameter('max_gap', 0.50) 

        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.marker_pub = self.create_publisher(MarkerArray, '/garden_features', 10)
        
        self.is_processing = False
        self.max_gap = self.get_parameter('max_gap').value

    def find_intersection(self, line1, line2):
        p1, p2 = line1; p3, p4 = line2
        x1, y1 = p1; x2, y2 = p2; x3, y3 = p3; x4, y4 = p4
        denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
        if denom == 0: return None 
        ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom
        return np.array([x1 + ua * (x2 - x1), y1 + ua * (y2 - y1)])

    def get_lines_with_gap_check(self, points):
        """Findet die beste Linie und teilt sie bei Lücken auf."""
        if len(points) < 6: return [], None
        
        pts = points.astype(np.float32)
        best_mask = None
        best_count = 0
        dist_thresh = 0.05

        # 1. RANSAC wie gehabt
        for _ in range(40):
            idx = np.random.choice(len(pts), 2, replace=False)
            p1, p2 = pts[idx[0]], pts[idx[1]]
            vec = p2 - p1
            norm = np.linalg.norm(vec)
            if norm < 0.01: continue 
            normal = np.array([-vec[1], vec[0]]) / norm
            dists = np.abs(np.dot(pts - p1, normal))
            mask = dists < dist_thresh
            count = np.sum(mask)
            if count > best_count:
                best_count = count
                best_mask = mask

        if best_count < 6: return [], None

        # 2. Inlier verfeinern
        inliers = points[best_mask]
        mean = np.mean(inliers, axis=0)
        uu, dd, vv = np.linalg.svd(inliers - mean)
        direction = vv[0]
        
        # 3. GAP-CHECK (Tor-Erkennung)
        projections = np.dot(inliers - mean, direction)
        sort_idx = np.argsort(projections)
        sorted_inliers = inliers[sort_idx]
        sorted_proj = projections[sort_idx]

        # Abstände zwischen aufeinanderfolgenden Punkten berechnen
        diffs = np.linalg.norm(sorted_inliers[1:] - sorted_inliers[:-1], axis=1)
        # Wo ist die Lücke größer als max_gap?
        gap_indices = np.where(diffs > self.max_gap)[0]

        split_lines = []
        last_idx = 0
        
        # Wand an den Lücken zerteilen
        for gap_idx in gap_indices:
            segment = sorted_inliers[last_idx : gap_idx + 1]
            if len(segment) >= 5: # Nur Segmente mit genug Punkten nehmen
                p_start = segment[0]
                p_end = segment[-1]
                split_lines.append((p_start, p_end))
            last_idx = gap_idx + 1
        
        # Das letzte (oder einzige) Stück hinzufügen
        final_segment = sorted_inliers[last_idx:]
        if len(final_segment) >= 5:
            split_lines.append((final_segment[0], final_segment[-1]))

        return split_lines, best_mask

    def scan_callback(self, msg):
        if self.is_processing: return
        self.is_processing = True

        start_zeit = time.perf_counter() # Zeitnahme startet
        ranges = np.array(msg.ranges)
        valid = np.isfinite(ranges) & (ranges > 0.1) & (ranges < 30.0)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        points = np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))[valid]

        all_detected_walls = []
        temp_points = points.copy()

        # Iterativ Linien suchen
        for _ in range(10):
            if len(temp_points) < 6: break
            lines, mask = self.get_lines_with_gap_check(temp_points)
            if not lines: break
            
            all_detected_walls.extend(lines)
            temp_points = temp_points[~mask]

        # Visualisierung (Stabil mit festen IDs)
        markers = MarkerArray()
        # Vorhandene Marker löschen (einfachste Art für Tor-Visualisierung)
        m_del = Marker()
        m_del.header.frame_id = "lidar"
        m_del.header.stamp = self.get_clock().now().to_msg()
        m_del.action = 3 # DELETEALL
        markers.markers.append(m_del)

        for i, (start, end) in enumerate(all_detected_walls):
            m = Marker()
            m.header.frame_id = "lidar"  # ODER msg.header.frame_id vom LaserScan
            m.header.stamp = self.get_clock().now().to_msg() # Aktuelle Zeit setzen!
            
            #m.header = msg.header
            m.ns = "walls"
            m.id = i
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.15; m.color.g = 1.0; m.color.a = 0.8
            m.points = [Point(x=float(start[0]), y=float(start[1]), z=-0.1), 
                        Point(x=float(end[0]), y=float(end[1]), z=-0.1)]
            markers.markers.append(m)

        self.marker_pub.publish(markers)
        self.is_processing = False
        
        ende_zeit = time.perf_counter() # Zeitnahme endet
        dauer_ms = (ende_zeit - start_zeit) * 1000 # Umrechnung in Millisekunden
        
        # Ausgabe im Terminal (alle Sekunde, um das Terminal nicht zu fluten)
        self.get_logger().info(f"⏱️ Rechenzeit: {dauer_ms:.2f} ms", throttle_duration_sec=1.0)

def main():
    rclpy.init()
    node = SmartCornerDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()