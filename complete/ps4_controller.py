# ps4_controller.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame

class PS4ControllerNode(Node):
    def __init__(self):
        super().__init__('ps4_controller_node')
        self.publisher_ = self.create_publisher(String, 'target_position', 10)
        
        # Initialize pygame
        pygame.init()
        pygame.joystick.init()
        
        # Wait for controller
        while pygame.joystick.get_count() == 0:
            self.get_logger().info('Waiting for PS4 controller...')
            pygame.joystick.quit()
            pygame.joystick.init()
            
        # Initialize controller
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        self.get_logger().info(f'Connected to controller: {self.controller.get_name()}')
        
        # Create timer for controller updates
        self.create_timer(0.02, self.publish_controller)  # 50Hz
        
        # Current position tracking
        self.current_x = 0.0
        self.current_y = 0.0
        self.speed = 0.001  # Movement speed
        
    def publish_controller(self):
        pygame.event.pump()
        
        # Get joystick values
        x = self.controller.get_axis(0)  # Left stick X
        y = -self.controller.get_axis(1)  # Left stick Y (inverted)
        
        # Update position
        self.current_x += x * self.speed
        self.current_y += y * self.speed
        
        # Publish position
        msg = String()
        msg.data = f"{self.current_x:.3f} {self.current_y:.3f}"
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PS4ControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()