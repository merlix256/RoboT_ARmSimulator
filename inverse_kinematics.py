import numpy as np
import math

class InverseKinematics:
    def __init__(self, joint_lengths):
        """
        Initialize the InverseKinematics solver.
        
        Args:
            joint_lengths (list): List of joint lengths [l1, l2, l3]
        """
        self.joint_lengths = joint_lengths
        self.l1, self.l2, self.l3 = joint_lengths
        self.current_angles = [0, 0, 0]

    def calculate_inverse_kinematics(self, target_pos):
        """
        Calculate inverse kinematics for a 3-joint robotic arm.
        
        Args:
            target_pos (tuple): Target position (x, y)
            
        Returns:
            list: Joint angles in degrees [angle1, angle2, angle3]
            
        Raises:
            ValueError: If target position is out of reach
        """
        x_target, y_target = target_pos
        d = math.sqrt(x_target**2 + y_target**2)

        # Check if target is within reach
        if d > sum(self.joint_lengths):
            raise ValueError("Target position is out of reach.")

        # Reduce to 2-link IK problem by subtracting last segment
        reduced_d = d - self.l3
        if reduced_d < 0:
            reduced_d = 0

        try:
            # Calculate angle2 using cosine law
            cos_angle2 = (reduced_d**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
            if cos_angle2 < -1 or cos_angle2 > 1:
                raise ValueError("No valid solution exists for these joint lengths and target.")
            
            angle2 = math.acos(cos_angle2)

            # Calculate angle1
            angle1 = math.atan2(y_target, x_target) - math.atan2(
                self.l2 * math.sin(angle2),
                self.l1 + self.l2 * math.cos(angle2)
            )

            # Calculate angle3
            angle3 = math.atan2(y_target, x_target) - angle1 - angle2

            # Convert to degrees
            angles = [
                math.degrees(angle1),
                math.degrees(angle2),
                math.degrees(angle3)
            ]

            return angles

        except ValueError as e:
            raise ValueError(f"IK calculation failed: {str(e)}")

    def interpolate_to_target(self, target_angles, step_size=5):
        """
        Interpolate current angles towards target angles.
        
        Args:
            target_angles (list): Target angles in degrees
            step_size (float): Maximum step size in degrees
            
        Returns:
            list: New interpolated angles
            bool: True if target is reached
        """
        new_angles = []
        target_reached = True

        for current, target in zip(self.current_angles, target_angles):
            if abs(target - current) > step_size:
                new_angle = current + step_size * np.sign(target - current)
                target_reached = False
            else:
                new_angle = target
            new_angles.append(new_angle)

        self.current_angles = new_angles
        return new_angles, target_reached

    def get_current_angles(self):
        """
        Get current joint angles.
        
        Returns:
            list: Current angles in degrees
        """
        return self.current_angles

    def reset_angles(self):
        """Reset all angles to zero."""
        self.current_angles = [0, 0, 0]