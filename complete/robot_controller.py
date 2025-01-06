# robot_controller.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math
import numpy as np

class RobotArmController(Node):
    def __init__(self):
        super().__init__('robot_arm_controller')
        
        # Create subscribers and publishers
        self.pos_subscription = self.create_subscription(
            String,
            'target_position',
            self.position_callback,
            10
        )
        self.angle_publisher = self.create_publisher(
            String,
            'joint_angles',
            10
        )
        
        # Robot parameters
        self.L1 = 0.1  # Length of first link
        self.L2 = 0.08  # Length of second link
        self.L3 = 0.05  # Length of third link
        
        # Current state
        self.current_angles = [90, 90, 90]  # Starting angles
        
    def calculate_inverse_kinematics(self, x, y):
        try:
            # Calculate distance to target
            target_distance = math.sqrt(x*x + y*y)
            
            # Check if point is reachable
            if target_distance > (self.L1 + self.L2 + self.L3):
                raise ValueError("Target position out of reach")
                
            # Calculate first joint angle (base rotation)
            theta1 = math.atan2(y, x)
            
            # Calculate second joint angle
            d = math.sqrt(x*x + y*y) - self.L3  # Adjust for third link
            cos_theta2 = (d*d - self.L1*self.L1 - self.L2*self.L2) / (2*self.L1*self.L2)
            cos_theta2 = min(1, max(-1, cos_theta2))  # Clamp to valid range
            theta2 = math.acos(cos_theta2)
            
            # Calculate third joint angle
            theta3 = math.atan2(y, x) - theta1 - theta2
            
            # Convert to degrees
            angles = [
                math.degrees(theta1),
                math.degrees(theta2),
                math.degrees(theta3)
            ]
            
            return angles
            
        except Exception as e:
            self.get_logger().error(f'Inverse kinematics error: {e}')
            return None
        
    def position_callback(self, msg):
        try:
            # Parse target position
            x, y = map(float, msg.data.split())
            
            # Calculate joint angles
            angles = self.calculate_inverse_kinematics(x, y)
            
            if angles:
                # Publish joint angles
                msg = String()
                msg.data = f"{angles[0]:.1f} {angles[1]:.1f} {angles[2]:.1f}"
                self.angle_publisher.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f'Error processing position: {e}')

def main(args=None):
    rclpy.init(args=args)
    controller = RobotArmController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()