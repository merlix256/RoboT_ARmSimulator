import serial
import time
import logging

class ArduinoController:
    def __init__(self, port='/dev/ttyUSB0', baud_rate=9600):
        self.logger = logging.getLogger(__name__)
        try:
            self.serial = serial.Serial(port, baud_rate, timeout=1)
            time.sleep(2)  # Arduino reset delay
            self.logger.info(f"Connected to Arduino on {port}")
        except serial.SerialException as e:
            self.logger.error(f"Failed to connect to Arduino: {e}")
            raise

    def send_servo_angles(self, angles):
        if len(angles) != 3:
            raise ValueError("Expected 3 angles")
        
        try:
            # Constrain angles to 0-180
            constrained_angles = [max(0, min(180, int(a))) for a in angles]
            command = f"{constrained_angles[0]},{constrained_angles[1]},{constrained_angles[2]}\n"
            self.serial.write(command.encode())
            self.logger.debug(f"Sent angles: {constrained_angles}")
            
            # Wait for acknowledgment
            response = self.serial.readline().decode().strip()
            if response != "OK":
                self.logger.warning(f"Unexpected response: {response}")
                return False
            return True
        except Exception as e:
            self.logger.error(f"Error sending angles: {e}")
            return False

    def close(self):
        try:
            self.serial.close()
            self.logger.info("Arduino connection closed")
        except Exception as e:
            self.logger.error(f"Error closing connection: {e}")