import numpy as np
import math
import time
import pygame
import csv
from robot_arm_visualize_with_metric import RobotArmVisualizer

# Function to calculate inverse kinematics for a 3-joint robotic arm
def inverse_kinematics_3d(joint_lengths, target_pos):
    x_target, y_target = target_pos
    l1, l2, l3 = joint_lengths
    d = math.sqrt(x_target**2 + y_target**2)

    if d > (l1 + l2 + l3):
        raise ValueError("Target position is out of reach.")

    reduced_d = d - l3
    if reduced_d < 0:
        reduced_d = 0

    cos_angle2 = (reduced_d**2 - l1**2 - l2**2) / (2 * l1 * l2)
    angle2 = math.acos(cos_angle2)
    angle1 = math.atan2(y_target, x_target) - math.atan2(l2 * math.sin(angle2), l1 + l2 * math.cos(angle2))
    angle3 = math.atan2(y_target, x_target) - angle1 - angle2

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

# Function to read positions and angles from CSV
def read_csv(filename):
    positions_and_angles = []
    with open(filename, mode='r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip header if there is one
        for row in reader:
            x_pos, y_pos, angle1, angle2, angle3 = map(float, row)
            positions_and_angles.append((x_pos, y_pos, angle1, angle2, angle3))
    return positions_and_angles

# Main script
if __name__ == "__main__":
    joint_lengths = [0.1, 0.08, 0.05]  # Lengths for a 3-joint arm
    visualizer = RobotArmVisualizer(
        joint_lengths=joint_lengths,
        pixels_per_meter=1000,
        grid_size_meters=0.1,
    )

    # Read positions and angles from the CSV file
    positions_and_angles = read_csv('robot_arm_data.csv')
    
    # Ask the user for the mode (angle-based or position-based)
    mode = input("Choose visualization mode (angles/positions): ").strip().lower()
    if mode not in ['angles', 'positions']:
        print("Invalid choice! Defaulting to 'positions'.")
        mode = 'positions'

    current_target_index = 0
    current_angles = [0, 0, 0]  # Initial angles for the arm
    step_size = 5  # Degrees per frame for smooth motion

    while visualizer.update():
        # Get the current target data from the CSV
        x_pos, y_pos, angle1, angle2, angle3 = positions_and_angles[current_target_index]

        if mode == 'positions':
            # Calculate angles using inverse kinematics based on the target position
            target_pos = (x_pos, y_pos)
            try:
                target_angles = inverse_kinematics_3d(joint_lengths, target_pos)
                current_angles = interpolate_angles(current_angles, target_angles, step=step_size)
            except ValueError as e:
                print(f"Error: {e}")
                current_target_index = (current_target_index + 1) % len(positions_and_angles)
                continue

        elif mode == 'angles':
            # Directly use the angles from the CSV
            target_angles = [angle1, angle2, angle3]

            # Update the robot arm with the target angles directly
            visualizer.update_angles(target_angles)
        
        # Always update the robot arm with the current interpolated angles (even in angles mode)
        visualizer.update_angles(current_angles)

        # Get the actual end-effector position from the visualizer
        e_pos = visualizer.get_end_effector_position()

        # Print current state
        print(f"Target: ({x_pos}, {y_pos}), End effector: {e_pos}, Current angles: {current_angles}")

        # Check if the target is reached (use a small threshold to check error)
        error = np.linalg.norm(np.array([x_pos, y_pos]) - np.array(e_pos))
        if error < 0.01:  # If within 1cm of the target
            print(f"Reached target with error {error:.4f} meters.")
            time.sleep(1)  # Pause briefly at the target
            current_target_index = (current_target_index + 1) % len(positions_and_angles)

    pygame.quit()
