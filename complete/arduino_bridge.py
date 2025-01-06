import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import time
import numpy as np

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        
        # Initialize serial connection to Arduino
        self.serial_port = '/dev/ttyUSB0'  # Adjust this to match your Arduino port
        self.baud_rate = 115200
        
        # Store previous angles for change detection
        self.prev_angles = None
        self.angle_threshold = 0.5  # Minimum angle change to trigger update (in degrees)
        
        try:
            self.arduino = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1.0
            )
            time.sleep(2)  # Wait for Arduino to reset
            self.get_logger().info(f'Connected to Arduino on {self.serial_port}')
            
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            raise
        
        # Create subscription to receive angles
        self.angles_subscription = self.create_subscription(
            Float32MultiArray,
            'target_angles',
            self.angles_callback,
            10
        )
        
        self.get_logger().info('Arduino Bridge Node Started')
    
    def angles_have_changed(self, new_angles):
        """Check if angles have changed significantly"""
        if self.prev_angles is None:
            return True
            
        # Calculate maximum difference between current and previous angles
        max_diff = max(abs(n - p) for n, p in zip(new_angles, self.prev_angles))
        return max_diff > self.angle_threshold
    
    def angles_callback(self, msg):
        """Handle incoming angle messages and send to Arduino"""
        try:
            current_angles = list(msg.data)
            
            # Only process if angles have changed significantly
            if self.angles_have_changed(current_angles):
                # Format angles as a comma-separated string with newline
                angles_str = ','.join(f"{angle:.2f}" for angle in current_angles) + '\n'
                
                self.get_logger().info(f'New angles: {angles_str.strip()}')
                
                # Send to Arduino
                self.arduino.write(angles_str.encode())
                self.arduino.flush()
                
                # Update previous angles
                self.prev_angles = current_angles
                
                # Read acknowledgment from Arduino
                while self.arduino.in_waiting:
                    response = self.arduino.readline().decode().strip()
                    if response and not response.startswith("Arduino"):  # Filter out startup messages
                        self.get_logger().info(f'Arduino response: {response}')
                    
        except serial.SerialException as e:
            self.get_logger().error(f'Serial communication error: {e}')
        except Exception as e:
            self.get_logger().error(f'Error sending angles to Arduino: {e}')
    
    def destroy_node(self):
        """Clean up serial connection when node is shut down"""
        if hasattr(self, 'arduino'):
            self.arduino.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    bridge = ArduinoBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()