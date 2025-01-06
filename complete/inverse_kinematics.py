import math
import numpy as np

class InverseKinematics3D:
    def __init__(self, joint_lengths):
        """
        Initialize the Inverse Kinematics class with the lengths of the arm's joints.
        :param joint_lengths: List of lengths for each joint (e.g., [l1, l2, l3])
        """
        self.joint_lengths = joint_lengths
    def calculate_error(self, current_pos, target_pos):
        """
        Calculate the Euclidean distance (error) between the current end-effector
        position and the target position.

        :param current_pos: Tuple (x, y) of the current end-effector position
        :param target_pos: Tuple (x, y) of the target position
        :return: The Euclidean distance between the two positions
        """
        x_current, y_current = current_pos
        x_target, y_target = target_pos
        error = math.sqrt((x_current - x_target)**2 + (y_current - y_target)**2)
        return error
    
    def calculate(self, target_pos):
        """
        Calculate the inverse kinematics for a 3-joint robotic arm.
        :param target_pos: Target position (x, y) for the end effector.
        :return: List of joint angles [angle1, angle2, angle3] in degrees.
        """
        x_target, y_target = target_pos
        l1, l2, l3 = self.joint_lengths
        d = math.sqrt(x_target**2 + y_target**2)

        # Check if the target is within reach
        if d > (l1 + l2 + l3):
            raise ValueError("Target position is out of reach.")

        # Simplified 3-joint IK assuming planar motion
        reduced_d = d - l3
        if reduced_d < 0:
            reduced_d = 0  # Clamp to valid range

        cos_angle2 = (reduced_d**2 - l1**2 - l2**2) / (2 * l1 * l2)
        angle2 = math.acos(cos_angle2)
        angle1 = math.atan2(y_target, x_target) - math.atan2(l2 * math.sin(angle2), l1 + l2 * math.cos(angle2))
        angle3 = math.atan2(y_target, x_target) - angle1 - angle2

        # Convert to degrees
        angle1_deg = math.degrees(angle1)
        angle2_deg = math.degrees(angle2)
        angle3_deg = math.degrees(angle3)

        return [angle1_deg, angle2_deg, angle3_deg]

def interpolate_angles(current_angles, target_angles, step=1):
    """
    Interpolate between current and target angles for smooth motion.
    :param current_angles: List of current joint angles.
    :param target_angles: List of target joint angles.
    :param step: Step size in degrees for each frame.
    :return: List of interpolated angles.
    """
    return [
        current + step * np.sign(target - current) if abs(target - current) > step else target
        for current, target in zip(current_angles, target_angles)
    ]
