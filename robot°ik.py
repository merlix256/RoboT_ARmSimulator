from robot_arm_visualize_with_metric import RobotArmVisualizer
import numpy as np
import math
import time
import pygame
# Function to calculate inverse kinematics for a 3-joint robotic arm
def inverse_kinematics_3d(joint_lengths, target_pos):
    x_target, y_target = target_pos
    l1, l2, l3 = joint_lengths
    d = math.sqrt(x_target**2 + y_target**2)

    # Check if the target is within reach
    if d > (l1 + l2 + l3):
        raise ValueError("Target position is out of reach.")

    # Simplified 3-joint IK assuming planar motion
    # First reduce to a 2-link IK problem by subtracting the last segment
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

# Interpolate angles for smoother motion
def interpolate_angles(current_angles, target_angles, step=1):
    return [
        current + step * np.sign(target - current) if abs(target - current) > step else target
        for current, target in zip(current_angles, target_angles)
    ]

# Main script
if __name__ == "__main__":
    joint_lengths = [0.1, 0.08, 0.05]  # Lengths for a 3-joint arm
    visualizer = RobotArmVisualizer(
        joint_lengths=joint_lengths,
        pixels_per_meter=1000,
        grid_size_meters=0.1,
    )

    target_positions = [
        (0.2, 0.0),  # Target 1
        (0.1, 0.15), # Target 2
        (-0.1, 0.1), # Target 3
        (0.0, -0.2), # Target 4
    ]
    current_target_index = 0
    current_angles = [0, 0, 0]  # Initial angles for the arm
    step_size = 5  # Degrees per frame for smooth motion

    while visualizer.update():
        # Get the current target position
        target_pos = target_positions[current_target_index]

        try:
            # Calculate target angles using IK
            target_angles = inverse_kinematics_3d(joint_lengths, target_pos)

            # Interpolate angles for smoother motion
            current_angles = interpolate_angles(current_angles, target_angles, step=step_size)

            # Update the robot arm with interpolated angles
            visualizer.update_angles(current_angles)

            # Get the actual end-effector position from the visualizer
            e_pos = visualizer.get_end_effector_position()

            # Print current state
            print(f"Target: {target_pos}, End effector: {e_pos}, Current angles: {current_angles}")

            # Check if the target is reached
            error = np.linalg.norm(np.array(target_pos) - np.array(e_pos))
            if error < 0.01:  # If within 1cm of the target
                print(f"Reached target with error {error:.4f} meters.")
                time.sleep(1)  # Pause briefly at the target
                current_target_index = (current_target_index + 1) % len(target_positions)

        except ValueError as e:
            print(f"Error: {e}")
            current_target_index = (current_target_index + 1) % len(target_positions)

    pygame.quit()
