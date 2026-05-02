import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
import pygame
import sys

class PygameTeleop(Node):
    def __init__(self):
        super().__init__('pygame_teleop')
        
        # Custom Profil: Wir akzeptieren nur den EINEN neuesten Wert.
        # Alles was älter ist, wird sofort gelöscht.
        custom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Schnell, keine Garantien (wie UDP)
            history=HistoryPolicy.KEEP_LAST,
            depth=1,                                    # <--- WICHTIG: Puffergröße auf 1 zwingen!
            durability=DurabilityPolicy.VOLATILE)

        # Topic-Name (hier anpassen, falls nötig)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', custom_qos)
        
        # Timer veröffentlicht 10x pro Sekunde
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.twist = Twist()
        
        self.speed_linear = 0.5   
        self.speed_angular = 1.0  
        self.linear_step = 0.1
        self.angular_step = 0.2

    def timer_callback(self):
        self.publisher_.publish(self.twist)

    def stop_robot(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publisher_.publish(self.twist)

    def update_twist(self, keys):
        """Wertet den physischen Zustand der Tasten aus (ignoriert Auto-Repeat)"""
        # Linear (Vor / Zurück)
        if keys[pygame.K_UP] and not keys[pygame.K_DOWN]:
            self.twist.linear.x = self.speed_linear
        elif keys[pygame.K_DOWN] and not keys[pygame.K_UP]:
            self.twist.linear.x = -self.speed_linear
        else:
            self.twist.linear.x = 0.0

        # Angular (Links / Rechts drehen)
        if keys[pygame.K_LEFT] and not keys[pygame.K_RIGHT]:
            self.twist.angular.z = self.speed_angular
        elif keys[pygame.K_RIGHT] and not keys[pygame.K_LEFT]:
            self.twist.angular.z = -self.speed_angular
        else:
            self.twist.angular.z = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = PygameTeleop()

    pygame.init()
    
    width, height = 400, 320
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("ROS 2 Pygame Teleop")
    
    font = pygame.font.SysFont("monospace", 18, bold=True)
    small_font = pygame.font.SysFont("monospace", 14)

    clock = pygame.time.Clock()
    running = True

    while rclpy.ok() and running:
        rclpy.spin_once(node, timeout_sec=0.01)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                
                # Geschwindigkeiten anpassen (soll nur 1x pro Druck passieren)
                elif event.key == pygame.K_w:
                    node.speed_linear += node.linear_step
                elif event.key == pygame.K_s:
                    node.speed_linear = max(0.0, node.speed_linear - node.linear_step)
                elif event.key == pygame.K_a:
                    node.speed_angular += node.angular_step
                elif event.key == pygame.K_d:
                    node.speed_angular = max(0.0, node.speed_angular - node.angular_step)

        # HIER IST DIE MAGIE: Lese aus, welche Tasten aktuell gehalten werden
        keys = pygame.key.get_pressed()
        node.update_twist(keys)

        # UI Zeichnen
        screen.fill((30, 30, 40))
        
        lines = [
            (font, "🎮 ROS 2 Control Panel", (100, 200, 255)),
            (small_font, "-"*30, (150, 150, 150)),
            (small_font, "[PFEILTASTEN] : Fahren (Halten)", (200, 200, 200)),
            (small_font, "[W] / [S]     : Vorwärts-Speed +/-", (200, 200, 200)),
            (small_font, "[A] / [D]     : Dreh-Speed +/-", (200, 200, 200)),
            (small_font, "[ESC]         : Beenden", (255, 100, 100)),
            (small_font, "", (0, 0, 0)),
            (font, f"Linear : {node.speed_linear:.2f} m/s", (255, 255, 100)),
            (font, f"Angular: {node.speed_angular:.2f} rad/s", (255, 255, 100)),
        ]

        y_offset = 20
        for text_font, text, color in lines:
            screen.blit(text_font.render(text, True, color), (20, y_offset))
            y_offset += 25

        # Ampel-Indikator
        is_moving = node.twist.linear.x != 0 or node.twist.angular.z != 0
        status_color = (100, 255, 100) if is_moving else (255, 100, 100)
        screen.blit(font.render("STATUS: FAHRT" if is_moving else "STATUS: STOP", True, status_color), (20, y_offset + 10))

        pygame.display.flip()
        clock.tick(30)

    # Cleanup
    node.stop_robot()
    pygame.quit()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)

if __name__ == '__main__':
    main()
    