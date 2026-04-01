import rclpy
from rclpy.node import Node
import numpy as np
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
import smbus2
import time
import json

I2C_BUS = 7
MAG_ADDR = 0x30


class CompassCalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration')
                
        # Publisher für die Fahrbefehle
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Publisher für rviz Textausgabe:
        self.text_pub = self.create_publisher(Marker, 'text_marker_topic', 10)

        # Services erstellen: Typ (Trigger)
        self.srv = self.create_service(Trigger, 'Kompass_Kalibrierung',            self.CompassCalibrationCallBack)

        # Erstelle einen Publisher für den Kalibrierungs-Status
        self.calib_pub = self.create_publisher(Bool, '/compass_calibration', 10)

    def SetCompassCalibrationRunning(self, status: bool):
        # Sende den neuen Status ans Netzwerk
        msg = Bool()
        msg.data = status
        self.calib_pub.publish(msg)
        self.get_logger().info(f"Sende Kalibrierungs-Status: {status}")

        
    def CompassCalibrationCallBack(self, request, response):
        self.SetCompassCalibrationRunning(True)
        circleMeasurements = 600
        zOffsetMeasurements = 100
        measCounter = 0
        self.SetVelocities(omega=0.2, vLinear=0)
        
        self.bus = smbus2.SMBus(I2C_BUS)

        # Sensor vor der Messung aufwecken und entmagnetisieren!
        print("Initialisiere Sensor (SET-Puls)...")
        self.init_mag()
        
        points_xy = []
        
        
        # PHASE 1: XY-Kreise
        print("Kalibrierung PHASE 1: Langsame 360° Kreise auf FLACHEM Boden fahren")
        
        for i in range(circleMeasurements):
            measCounter += 1
            points_xy.append(self.read_mag_raw())
            self.RvizPrint(f"Kalibrierung Phase 1: Kreisfahrt - Punkte gesammelt: {measCounter}/{circleMeasurements}")
            time.sleep(0.05)


        points_xy = np.array(points_xy)
        center_xy, T_xy = self.fit_2d_ellipse(points_xy[:, 0], points_xy[:, 1])
        
        # Radius R berechnen: Transformiere Punkte und nimm den mittleren Abstand zum Zentrum
        # Das ist genauer als eine Konstante, da es das lokale Feld misst
        transformed_points = (points_xy[:, :2] - center_xy) @ T_xy.T
        R = np.mean(np.linalg.norm(transformed_points, axis=1))
        
        # PHASE 2: Z-Offset (Statisches Messen)
        self.SetVelocities(0, 0)
        self.RvizPrint(f"PHASE 1 beendet. Berechneter XY-Radius R: {R:.1f}")
        z_vals = []
        measCounter = 0
        
        # Warte bis E-Karren steht
        time.sleep(0.5)

        self.RvizPrint(f"Kalibrierung Phase 2: Z-Offset-Messung über {zOffsetMeasurements} Messpunkte")
        for _ in range(zOffsetMeasurements):
            z_vals.append(self.read_mag_raw()[2])
            time.sleep(0.02)
            measCounter += 1

        self.SetCompassCalibrationRunning(False)
        z_raw_avg = np.mean(z_vals)
        
        # Z-Offset über Inklination (65° für Deutschland)
        inclination = math.radians(65.0)
        z_earth_ideal = R * math.tan(inclination)
        
        if z_raw_avg < 0:
            z_earth_ideal = -z_earth_ideal
            
        z_offset = z_raw_avg - z_earth_ideal
        
        # Ergebnisse speichern
        center_3d = [float(center_xy[0]), float(center_xy[1]), float(z_offset)]
        matrix_3d = [
            [float(T_xy[0,0]), float(T_xy[0,1]), 0.0],
            [float(T_xy[1,0]), float(T_xy[1,1]), 0.0],
            [0.0,              0.0,              1.0]
        ]
        
        with open('mag_ellipsoid.json', 'w') as f:
            json.dump({
                'offset': center_3d, 
                'matrix': matrix_3d,
                'raw_points': points_xy.tolist()
            }, f)
            
        print("\n--- Kalibrierung fertig ---")
        print(f"Zentrum XY: {center_xy}")
        print(f"Z-Offset:   {z_offset:.1f}")
        print(f"Radius R:   {R:.1f}")
        print("Gespeichert in mag_ellipsoid.json")

        response.success = True 
        response.message = "Gespeichert in mag_ellipsoid.json"
        return response

    def SetVelocities(self, omega, vLinear):
        drive_msg = Twist()
        drive_msg.linear.x = float(vLinear)
        drive_msg.angular.z = float(omega)
        self.cmd_pub.publish(drive_msg)
                
    def init_mag(self):
        # Register 0x08 (Internal Control 0)
        # Bit 3 (0x08) sendet einen SET-Puls in die interne Spule
        self.bus.write_byte_data(MAG_ADDR, 0x08, 0x08)
        time.sleep(0.05) # Kurz warten, bis der Kondensator sich entladen hat
        
        # Optional: Einmal blind lesen, um alte Daten aus dem Puffer zu spülen
        self.bus.write_byte_data(MAG_ADDR, 0x08, 0x01)
        time.sleep(0.02)
        self.bus.read_i2c_block_data(MAG_ADDR, 0x00, 6)

    def read_mag_raw(self):
        self.bus.write_byte_data(MAG_ADDR, 0x08, 0x01)
        while not (self.bus.read_byte_data(MAG_ADDR, 0x07) & 0x01): 
            time.sleep(0.005)
        data = self.bus.read_i2c_block_data(MAG_ADDR, 0x00, 6)
        # MMC5883MA: Unsigned 16-bit, Nullpunkt bei 32768
        return np.array([
            ((data[1] << 8) | data[0]) - 32768,
            ((data[3] << 8) | data[2]) - 32768,
            ((data[5] << 8) | data[4]) - 32768
        ])

    def fit_2d_ellipse(self, x, y):
        # Ellipsen-Fit: Ax^2 + By^2 + Cxy + Dx + Ey = 1
        D_mat = np.column_stack((x**2, y**2, x*y, x, y))
        v = np.linalg.lstsq(D_mat, np.ones_like(x), rcond=None)[0]
        
        A_mat = np.array([[v[0], v[2]/2], [v[2]/2, v[1]]])
        b_vec = np.array([v[3], v[4]])
        
        # Offset (Hard-Iron XY)
        center_xy = -0.5 * np.linalg.solve(A_mat, b_vec)
        
        # Soft-Iron Matrix (T)
        evals, evecs = np.linalg.eigh(A_mat)
        # Korrigierte Gain-Berechnung (Wurzel aus evals)
        gain = np.sqrt(np.abs(evals))
        T_xy = evecs @ np.diag(gain) @ evecs.T
        
        # Skalierung: Wir berechnen den mittleren Radius R aus den Eigenwerten
        # det(A) hängt direkt mit der Fläche der Ellipse zusammen
        # Der Radius R des korrigierten Kreises ergibt sich aus 1/sqrt(det(T_xy))
        # Aber wir normalisieren T_xy hier erst am Ende.
        
        # Normalisierung der Matrix (det=1)
        T_xy /= np.sqrt(np.linalg.det(T_xy))
        
        # Radius-Bestimmung für Z: 
        # Nach der Transformation (p - center) @ T.T liegen die Punkte auf einem Kreis.
        # Wir berechnen den durchschnittlichen Radius der transformierten Punkte.
        return center_xy, T_xy

    def RvizPrint(self, text):
        marker = Marker()
        marker.header.frame_id = "map"  # Muss mit deinem Fixed Frame in RViz übereinstimmen
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Inhalt des Strings
        marker.text = f"{text}"
        #self.get_logger().info(text)
        
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
    # ROS2 Initialisierung
    rclpy.init()
    node = CompassCalibrationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Wird ausgelöst, wenn du Ctrl+C drückst
        node.get_logger().info("Node calibration wird durch Benutzer abgebrochen...")
    except Exception as e:
        # Fängt unerwartete Fehler ab
        node.get_logger().error(f"Unerwarteter Fehler: {e}")
    finally:
        # Dieser Block wird IMMER ausgeführt, egal ob Fehler oder Ctrl+C
        node.get_logger().info("Bereinige Ressourcen...")
               
        # Nur zerstören, wenn der Kontext noch gültig ist
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        
        # Optional: Komplettes Beenden erzwingen (hilft bei WSL2-Hängern)
        #sys.exit(0)

    
if __name__ == '__main__':
    main()