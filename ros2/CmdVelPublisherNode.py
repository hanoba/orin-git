import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('simple_teleop')
        # Publisher für das Topic /cmd_vel erstellen
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Teleop Node gestartet. Befehle eingeben...')
        self.run_loop()

    def run_loop(self):
        print("\n--- Manuelle Steuerung ---")
        print("Gib Werte ein (z.B. 0.5 für vorwärts, -0.2 für rückwärts)")
        
        try:
            while rclpy.ok():
                # Benutzereingabe abfragen
                try:
                    linear_x = float(input("Linear X (m/s): ") or 0.0)
                    angular_z = float(input("Angular Z (rad/s): ") or 0.0)
                except ValueError:
                    print("Bitte nur Zahlen eingeben!")
                    continue

                # Nachricht erstellen
                msg = Twist()
                msg.linear.x = linear_x
                msg.angular.z = angular_z

                # Nachricht senden
                self.publisher_.publish(msg)
                self.get_logger().info(f'Sende: Linear={linear_x}, Angular={angular_z}')
                
                print("---")
        except KeyboardInterrupt:
            # Stopp-Befehl beim Beenden senden
            stop_msg = Twist()
            self.publisher_.publish(stop_msg)
            self.get_logger().info('Stopp-Signal gesendet. Beende...')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    