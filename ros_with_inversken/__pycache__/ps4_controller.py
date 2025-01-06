# ps4_controller.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import math
from collections import deque
import numpy as np

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
        
        # Configuration
        self.max_reach = 0.15      # Maximum reach of the robot arm in meters
        self.dead_zone = 0.15      # Increased dead zone for better control
        self.sensitivity = 0.6     # Reduced sensitivity for finer control
        self.smoothing_window = 5  # Number of samples for moving average
        
        # Initialize position tracking
        self.current_x = 0.0
        self.current_y = 0.0
        
        # Buffers for smoothing
        self.x_buffer = deque([0.0] * self.smoothing_window, maxlen=self.smoothing_window)
        self.y_buffer = deque([0.0] * self.smoothing_window, maxlen=self.smoothing_window)
        
        # Create timer for reading controller input
        self.create_timer(0.02, self.read_controller)  # 50Hz update rate
        
        self.get_logger().info('PS4 Controller Node Started')
    
    def smooth_input(self, value, buffer):
        """Apply exponential smoothing to input"""
        buffer.append(value)
        return sum(buffer) / len(buffer)
    
    def apply_non_linear_scaling(self, value):
        """Apply non-linear scaling for better control at low speeds"""
        return math.copysign(abs(value) ** 1.5, value)
    
    def read_controller(self):
        pygame.event.pump()
        
        # Read raw stick values
        x = self.joystick.get_axis(0)
        y = -self.joystick.get_axis(1)  # Invert Y axis
        
        # Apply dead zone
        x = 0.0 if abs(x) < self.dead_zone else x
        y = 0.0 if abs(y) < self.dead_zone else y
        
        # Apply sensitivity and non-linear scaling
        if x != 0:
            x = self.apply_non_linear_scaling(x) * self.sensitivity
        if y != 0:
            y = self.apply_non_linear_scaling(y) * self.sensitivity
        
        # Smooth the input
        x = self.smooth_input(x, self.x_buffer)
        y = self.smooth_input(y, self.y_buffer)
        
        # Scale to robot arm reach
        target_x = x * self.max_reach
        target_y = y * self.max_reach
        
        # Update current position with momentum
        self.current_x = 0.8 * self.current_x + 0.2 * target_x  # 80% previous, 20% new
        self.current_y = 0.8 * self.current_y + 0.2 * target_y
        
        # Only publish if there's significant movement
        if abs(x) > 0.05 or abs(y) > 0.05:
            msg = String()
            msg.data = f"{self.current_x:.3f} {self.current_y:.3f}"
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
        pygame.quit()
        controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()