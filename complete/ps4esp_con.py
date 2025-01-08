import serial
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
ser.flush()

def parse_data(input_string):
    # Define a dictionary to hold parsed values
    parsed_data = {}

    # Regular expression patterns for each field
    patterns = {
        'idx': r"idx=(\d+)",
        'dpad': r"dpad:\s*(0x[0-9A-Fa-f]+)",
        'buttons': r"buttons:\s*(0x[0-9A-Fa-f]+)",
        'axis_L': r"axis L:\s*(-?\d+),\s*(-?\d+)",
        'axis_R': r"axis R:\s*(-?\d+),\s*(-?\d+)",
        'brake': r"brake:\s*(-?\d+)",
        'throttle': r"throttle:\s*(-?\d+)",
        'misc': r"misc:\s*(0x[0-9A-Fa-f]+)",
        'gyro': r"gyro x:\s*(-?\d+)\s*y:\s*(-?\d+)\s*z:\s*(-?\d+)",
        'accel': r"accel x:\s*(-?\d+)\s*y:\s*(-?\d+)\s*z:\s*(-?\d+)"
    }

    # Extract and store values in the dictionary
    for key, pattern in patterns.items():
        match = re.search(pattern, input_string)
        if match:
            if key in ['axis_L', 'axis_R', 'gyro', 'accel']:
                parsed_data[key] = tuple(map(int, match.groups()))
            else:
                parsed_data[key] = int(match.group(1), 16) if '0x' in match.group(1) else int(match.group(1))
    
    return parsed_data

class TargetPositionGUI(Node):
    def __init__(self):
        super().__init__('target_position_gui')

        self.position_publisher = self.create_publisher(String, 'target_position', 10)
        self.msg = String()

        self.target_x = 0.0
        self.target_y = 0.0
        self.step_size = 0.01
        self.dead_zone = 100  # Threshold for dead zone

        if ser.is_open:
            self.get_logger().info("Arduino connected successfully!")
        else:
            self.get_logger().warn("Arduino not connected!")

        self.read_arduino_data()

    def read_arduino_data(self):
        while rclpy.ok():
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').rstrip()
                parsed_result = parse_data(line)

                if 'axis_L' in parsed_result:
                    self.handle_left_stick(parsed_result['axis_L'])
                if 'axis_R' in parsed_result:
                    self.handle_right_stick(parsed_result['axis_R'])
                if 'buttons' in parsed_result:
                    self.handle_buttons(parsed_result['buttons'])
                if 'dpad' in parsed_result:
                    self.handle_dpad(parsed_result['dpad'])

                self.publish_target_position()

    def handle_left_stick(self, axis_values):
        x_axis, y_axis = axis_values

        if abs(x_axis) > self.dead_zone:
            self.target_x += (x_axis / 1000) * self.step_size
        if abs(y_axis) > self.dead_zone:
            # Negate y_axis to correct the direction
            self.target_y -= (y_axis / 1000) * self.step_size

    def handle_right_stick(self, axis_values):
        _, y_axis = axis_values

        if abs(y_axis) > self.dead_zone:
            self.step_size = 0.01 + (y_axis / 10000)  # Scale step size based on vertical axis
            # Log the updated step size
            self.get_logger().info(f"Step Size (Speed): {self.step_size:.5f}")

    def handle_buttons(self, button_state):
        # Circle (Button 1)
        if button_state & (1 << 1):
            self.target_x = 0.10
            self.target_y = 0.20

        # Triangle (Button 3) - Define custom behavior if needed
        if button_state & (1 << 3):
            pass

        # Square (Button 0) - Define custom behavior if needed
        if button_state & (1 << 0):
            pass

    def handle_dpad(self, dpad_state):
        # D-pad Up (1)
        if dpad_state == 1:
            self.target_y += self.step_size
        # D-pad Down (2)
        elif dpad_state == 2:
            self.target_y -= self.step_size
        # D-pad Left (8)
        elif dpad_state == 8:
            self.target_x -= self.step_size
        # D-pad Right (4)
        elif dpad_state == 4:
            self.target_x += self.step_size

        
    def publish_target_position(self):
        self.msg.data = f"{self.target_x:.3f} {self.target_y:.3f}"
        self.position_publisher.publish(self.msg)
        # Log the target position along with step size
        self.get_logger().info(f"Target Position: X = {self.target_x:.3f}, Y = {self.target_y:.3f}, Step Size: {self.step_size:.5f}")

def main(args=None):
    rclpy.init(args=args)
    target_position_gui = TargetPositionGUI()

    try:
        rclpy.spin(target_position_gui)
    except KeyboardInterrupt:
        pass

    target_position_gui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
