import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32
import numpy as np
import time
import sys

sys.path.append('../HostSim')
import params

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self.declare_parameter('distance_threshold', 0.05) 
        self.declare_parameter('min_points', 6)
        # NEU: Wenn Punkte weiter als 50cm auseinander liegen -> Tor/Lücke
        self.declare_parameter('max_gap', 5.50) 

        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.marker_pub = self.create_publisher(MarkerArray, '/garden_features', 10)
        
        # Erstellen der Subscription
        # 1. Typ: Float32 (muss zum Publisher passen)
        # 2. Topic: '/compass_heading'
        # 3. Callback: self.listener_callback
        # 4. QoS: qos_profile_sensor_data (MUSS zum Publisher passen)
        self.subCompass = self.create_subscription(
            Float32,
            '/compass_heading',
            self.compass_callback,
            qos_profile_sensor_data
        )
        self.is_processing = False
        self.max_gap = self.get_parameter('max_gap').value
        self.theta = 0.0
        self.K_head = 0.5
                
        # Publisher für die Fahrbefehle
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        

    def compass_callback(self, msg):
        self.theta = msg.data
                
        # Nach Norden ausrichten
        self.SetTheta(-np.pi/2)


    def SetTheta(self, wantedTheta):
        vLinear = 0.0
        e = wantedTheta - self.theta
        if abs(e) < 0.02: omega = 0.0
        else: omega = self.K_head*e
        drive_msg = Twist()
        drive_msg.linear.x = float(vLinear)
        drive_msg.angular.z = float(omega)
        self.cmd_pub.publish(drive_msg)


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


    def RansacLineDetection(self, points):
        all_detected_walls = []
        temp_points = points.copy()

        # Iterativ Linien suchen
        for _ in range(10):
            if len(temp_points) < 6: break
            lines, mask = self.get_lines_with_gap_check(temp_points)
            if not lines: break
            
            all_detected_walls.extend(lines)
            temp_points = temp_points[~mask]
        return all_detected_walls
    
    
    def PublishMarkers(self, all_detected_walls):
            # Visualisierung (Stabil mit festen IDs)
        markers = MarkerArray()
        # Vorhandene Marker löschen (einfachste Art für Tor-Visualisierung)
        m_del = Marker()
        m_del.header.frame_id = "lidar"
        m_del.header.stamp = self.get_clock().now().to_msg()
        m_del.action = 3 # DELETEALL
        markers.markers.append(m_del)

        current_time = self.get_clock().now().to_msg()
        frame_id = "lidar"
        z_height = -0.1 # Die Höhe, auf der alles gezeichnet wird

        for i, (start, end) in enumerate(all_detected_walls):
            # ============================================
            # Linien-Marker
            # ============================================
            m_line = Marker()
            m_line.header.frame_id = frame_id
            m_line.header.stamp = current_time
            m_line.ns = "walls_lines" # Namespace geändert zur Unterscheidung
            m_line.id = i
            m_line.type = Marker.LINE_STRIP
            m_line.action = Marker.ADD
            m_line.scale.x = 0.15  # Linienbreite
            m_line.color.g = 1.0; m_line.color.a = 0.8 # Grün, leicht transparent
            
            # Start- und Endpunkte für die Linie setzen
            p_start = Point(x=float(start[0]), y=float(start[1]), z=z_height)
            p_end = Point(x=float(end[0]), y=float(end[1]), z=z_height)
            m_line.points = [p_start, p_end]
            
            markers.markers.append(m_line)

            # ============================================
            # Marker für den Start-Kreis
            # ============================================
            m_start_sphere = Marker()
            m_start_sphere.header.frame_id = frame_id
            m_start_sphere.header.stamp = current_time
            m_start_sphere.ns = "walls_endpoints" # Neuer Namespace für die Punkte
            # Wir brauchen eine eindeutige ID. Trick: Wir nutzen gerade Zahlen für Starts.
            m_start_sphere.id = i * 2 
            m_start_sphere.type = Marker.SPHERE # Typ ist jetzt eine Kugel
            m_start_sphere.action = Marker.ADD
            
            # Größe der Kugel (Durchmesser in x, y, z)
            # Machen wir sie etwas dicker als die Linie (0.25m vs 0.15m)
            m_start_sphere.scale.x = 0.25
            m_start_sphere.scale.y = 0.25
            m_start_sphere.scale.z = 0.25
            
            # Farbe: Machen wir die Endpunkte ROT, damit sie auffallen
            m_start_sphere.color.r = 1.0
            m_start_sphere.color.a = 1.0 # Voll sichtbar
            
            # Position setzen (Sphären nutzen pose.position, nicht points!)
            m_start_sphere.pose.position = p_start
            # WICHTIG: Eine Orientierung muss gesetzt sein, auch wenn sie neutral ist
            m_start_sphere.pose.orientation.w = 1.0
            
            markers.markers.append(m_start_sphere)

            # ============================================
            # Marker für den End-Kreis (Kugel)
            # ============================================
            # Wir kopieren die Eigenschaften des Start-Markers und passen nur ID und Position an
            m_end_sphere = Marker()
            # Kopiere Header und generelle Infos vom Start-Marker
            m_end_sphere.header = m_start_sphere.header
            m_end_sphere.ns = m_start_sphere.ns
            m_end_sphere.type = m_start_sphere.type
            m_end_sphere.action = m_start_sphere.action
            m_end_sphere.scale = m_start_sphere.scale
            m_end_sphere.color = m_start_sphere.color
            m_end_sphere.pose.orientation = m_start_sphere.pose.orientation

            # ID: Ungerade Zahl für das Ende, damit sie eindeutig bleibt
            m_end_sphere.id = i * 2 + 1
            
            # Position auf den Endpunkt setzen
            m_end_sphere.pose.position = p_end
            
            markers.markers.append(m_end_sphere)

        self.marker_pub.publish(markers)

    
    def scan_callback(self, msg):
        if self.is_processing: return
        self.is_processing = True

        start_zeit = time.perf_counter() # Zeitnahme startet
        ranges = np.array(msg.ranges)
        valid = np.isfinite(ranges) & (ranges > params.LidarRangeMin) & (ranges < params.LidarRangeMax)
        #valid = np.isfinite(ranges) & (ranges > 0.1) & (ranges < 30.0)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        points = np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))[valid]

        all_detected_walls = self.RansacLineDetection(points)
        self.PublishMarkers(all_detected_walls)
        
        ende_zeit = time.perf_counter() # Zeitnahme endet
        dauer_ms = (ende_zeit - start_zeit) * 1000 # Umrechnung in Millisekunden
        
        # Ausgabe im Terminal (alle Sekunde, um das Terminal nicht zu fluten)
        self.get_logger().info(f"⏱️ Rechenzeit: {dauer_ms:.2f} ms  Theta={np.rad2deg(self.theta):.0f}°", throttle_duration_sec=1.0)

        self.is_processing = False

def main():
    rclpy.init()
    node = Navigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()