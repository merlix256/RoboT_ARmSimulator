# target_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

class TargetPositionPublisher(Node):
    def __init__(self):
        super().__init__('target_position_publisher')
        self.publisher_ = self.create_publisher(String, 'target_position', 10)
        self.get_logger().info('Target Position Publisher Node Started')

    def publish_target(self, x, y):
        msg = String()
        msg.data = f"{x} {y}"
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing target position: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    publisher = TargetPositionPublisher()
    
    try:
        while rclpy.ok():
            try:
                # Get input from user (x y format)
                user_input = input("Enter target position (x y) or 'q' to quit: ")
                if user_input.lower() == 'q':
                    break
                
                # Parse input
                x, y = map(float, user_input.split())
                publisher.publish_target(x, y)
                
            except ValueError:
                publisher.get_logger().error('Invalid input format. Use "x y" format (e.g., "0.05 -0.07")')
            
    except KeyboardInterrupt:
        pass
    
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()