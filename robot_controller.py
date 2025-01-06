from inverse_kinematics import InverseKinematics
from robot_arm_visualize_with_metric import RobotArmVisualizer
from arduino_controller import ArduinoController
import logging


class RobotController:
    def __init__(self, joint_lengths, arduino_port, simulation=True):
        self.logger = logging.getLogger(__name__)
        self.ik_solver = InverseKinematics(joint_lengths)
        self.arduino = ArduinoController(arduino_port) if arduino_port else None
        
        if simulation:
            self.visualizer = RobotArmVisualizer(
                joint_lengths=joint_lengths,
                pixels_per_meter=1000,
                grid_size_meters=0.1
            )
        else:
            self.visualizer = None

    def move_to_position(self, target_pos, speed=3):
        try:
            target_angles = self.ik_solver.calculate_inverse_kinematics(target_pos)
            
            if self.visualizer:
                current_angles, reached = self.ik_solver.interpolate_to_target(target_angles, speed)
                self.visualizer.update_angles(current_angles)
                
            if self.arduino:
                success = self.arduino.send_servo_angles(target_angles)
                if not success:
                    self.logger.error("Failed to send angles to Arduino")
                    return False
                
            return True
            
        except ValueError as e:
            self.logger.error(f"IK calculation failed: {e}")
            return False
        except Exception as e:
            self.logger.error(f"Movement failed: {e}")
            return False

    def update_visualization(self):
        if self.visualizer:
            return self.visualizer.update()
        return True

    def close(self):
        if self.arduino:
            self.arduino.close()