# ps4_controller.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import math

class PS4ControllerNode(Node):
    def __init__(self):
        super().__init__('ps4_controller_node')
        self.publisher_ = self.create_publisher(String, 'target_position', 10)
        
        # Initialize pygame for joystick
        pygame.init()
        pygame.joystick.init()
        
        # Wait for controller
        while pygame.joystick.get_count() == 0:
            self.get_logger().info('Waiting for PS4 controller...')
            pygame.joystick.quit()
            pygame.joystick.init()
            
        # Initialize the first joystick
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        
        # Verify it's a PS4 controller
        controller_name = self.joystick.get_name()
        self.get_logger().info(f'Connected to controller: {controller_name}')
        
        # Configuration
        self.max_reach = 0.15  # Maximum reach of the robot arm in meters
        self.dead_zone = 0.1   # Joystick dead zone to prevent drift
        
        # Create timer for reading controller input
        self.create_timer(0.02, self.read_controller)  # 50Hz update rate
        
        self.get_logger().info('PS4 Controller Node Started')
        
    def read_controller(self):
        pygame.event.pump()  # Process pygame events
        
        # Read left stick values (-1 to 1)
        x = self.joystick.get_axis(0)  # Left/Right on left stick
        y = -self.joystick.get_axis(1)  # Up/Down on left stick (inverted)
        
        # Apply dead zone
        if abs(x) < self.dead_zone:
            x = 0
        if abs(y) < self.dead_zone:
            y = 0
            
        # Convert joystick values to target position
        # Scale the values to the robot's reach
        target_x = x * self.max_reach
        target_y = y * self.max_reach
        
        # Only publish if stick is out of dead zone
        if abs(x) > self.dead_zone or abs(y) > self.dead_zone:
            msg = String()
            msg.data = f"{target_x:.3f} {target_y:.3f}"
            self.publisher_.publish(msg)
            self.get_logger().debug(f'Published target position: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    controller_node = PS4ControllerNode()
    
    try:
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        pygame.quit()
        controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()