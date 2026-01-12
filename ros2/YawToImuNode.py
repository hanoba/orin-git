import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32  # Oder Float64, je nach Ihrem Topic
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import math

class YawToImuNode(Node):
    def __init__(self):
        super().__init__('yaw_to_imu_converter')
        
        # PARAMETER
        self.declare_parameter('input_topic', '/isaac/yaw') # Ihr Float-Topic
        self.declare_parameter('output_topic', '/imu_data')
        self.declare_parameter('frame_id', 'base_link') # Frame des Roboters

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        # Subscriber (Float)
        self.subscription = self.create_subscription(
            Float32,
            input_topic,
            self.listener_callback,
            qos_profile_sensor_data)

        # Publisher (Imu)
        self.publisher_ = self.create_publisher(Imu, output_topic, 10)

    def listener_callback(self, msg):
        # 1. Den Float-Wert holen (Annahme: Wert ist in Radiant! Falls Grad: * math.pi / 180)
        yaw = msg.data 

        # 2. Leere IMU Nachricht erstellen
        imu_msg = Imu()
        
        # 3. Zeitstempel "fälschen" (wie beim Lidar besprochen), damit EKF glücklich ist
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id

        # 4. Euler-Yaw in Quaternion umwandeln (Nur Z-Achse dreht sich)
        # Formel für Rotation um Z-Achse:
        # x=0, y=0, z=sin(yaw/2), w=cos(yaw/2)
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = math.sin(yaw / 2.0)
        imu_msg.orientation.w = math.cos(yaw / 2.0)

        # 5. Covariance setzen (WICHTIG!)
        # -1 auf Diagonale heißt "keine Daten". Wir setzen 0 bei Index 0,4,8
        # Wir setzen eine kleine Varianz für die Z-Achse (Index 8), da wir dem Wert vertrauen
        imu_msg.orientation_covariance = [0.0] * 9
        imu_msg.orientation_covariance[8] = 0.01  # Vertrauen in Yaw
        
        # Restliche Daten ignorieren wir (-1 setzen, damit EKF sie ignoriert)
        imu_msg.angular_velocity_covariance[0] = -1.0
        imu_msg.linear_acceleration_covariance[0] = -1.0

        self.publisher_.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YawToImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()