# arduino_bridge.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        
        # Create subscriber
        self.subscription = self.create_subscription(
            String,
            'joint_angles',
            self.angle_callback,
            10
        )
        
        # Serial setup
        self.port = '/dev/ttyUSB0'  # Change to your Arduino port
        self.baud_rate = 115200
        
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=1
            )
            time.sleep(2)  # Wait for Arduino to initialize
            self.get_logger().info(f'Connected to Arduino on {self.port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            raise e
    
    def angle_callback(self, msg):
        try:
            # Parse angles
            angle1, angle2, angle3 = map(float, msg.data.split())
            
            # Send to Arduino
            arduino_msg = f"<{angle1:.1f},{angle2:.1f},{angle3:.1f}>\n"
            self.serial.write(arduino_msg.encode())
            
            # Read response if available
            if self.serial.in_waiting:
                response = self.serial.readline().decode().strip()
                self.get_logger().debug(f'Arduino response: {response}')
                
        except Exception as e:
            self.get_logger().error(f'Error sending to Arduino: {e}')
    
    def destroy_node(self):
        if hasattr(self, 'serial'):
            self.serial.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    bridge = ArduinoBridgeNode()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()