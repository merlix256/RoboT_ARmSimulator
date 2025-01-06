from robot_arm_visualize_with_metric import RobotArmVisualizer
import numpy as np
import math
import time
import pygame
import csv

# Function to calculate inverse kinematics for a 3-joint robotic arm
def inverse_kinematics_3d(joint_lengths, target_pos):
    x_target, y_target = target_pos
    l1, l2, l3 = joint_lengths
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

# Interpolate angles for smoother motion
def interpolate_angles(current_angles, target_angles, step=1):
    return [
        current + step * np.sign(target - current) if abs(target - current) > step else target
        for current, target in zip(current_angles, target_angles)
    ]

# Function to generate target positions along a circle
def generate_circle_points(center, radius, num_points=100):
    circle_points = []
    for i in range(num_points):
        angle = 2 * math.pi * i / num_points  # Angle in radians
        x = center[0] + radius * math.cos(angle)
        y = center[1] + radius * math.sin(angle)
        circle_points.append((x, y))
    return circle_points

# Function to generate target positions for a square
def generate_square_points(center, size):
    square_points = [
        (center[0] - size / 2, center[1] - size / 2),  # Bottom-left
        (center[0] + size / 2, center[1] - size / 2),  # Bottom-right
        (center[0] + size / 2, center[1] + size / 2),  # Top-right
        (center[0] - size / 2, center[1] + size / 2),  # Top-left
    ]
    return square_points

# Function to generate target positions for a triangle
def generate_triangle_points(center, size):
    triangle_points = [
        (center[0], center[1] + size / math.sqrt(3)),  # Top vertex
        (center[0] - size / 2, center[1] - size / (2 * math.sqrt(3))),  # Bottom-left vertex
        (center[0] + size / 2, center[1] - size / (2 * math.sqrt(3))),  # Bottom-right vertex
    ]
    return triangle_points

# Main script
if __name__ == "__main__":
    joint_lengths = [0.1, 0.08, 0.05]  # Lengths for a 3-joint arm
    visualizer = RobotArmVisualizer(
        joint_lengths=joint_lengths,
        pixels_per_meter=1000,
        grid_size_meters=0.1,
    )

    # Shape parameters
    shape_type = 'circle'  # Options: 'circle', 'square', 'triangle'
    center = (0.0, 0.15)  # Center of the shape
    size = 0.05            # Size for the shape

    # Generate shape points based on the selected shape
    if shape_type == 'circle':
        target_positions = generate_circle_points(center, size, 100)
    elif shape_type == 'square':
        target_positions = generate_square_points(center, size=0.05)
    elif shape_type == 'triangle':
        target_positions = generate_triangle_points(center, size)
    else:
        raise ValueError("Invalid shape type")

    current_angles = [0, 0, 0]  # Initial angles for the arm
    step_size = 5  # Degrees per frame for smooth motion
    current_target_index = 0

    while visualizer.update():
        # Get the current target position for the shape
        target_pos = target_positions[current_target_index]

        try:
            # Calculate target angles using IK
            target_angles = inverse_kinematics_3d(joint_lengths, target_pos)

            # Interpolate angles for smoother motion
            current_angles = interpolate_angles(current_angles, target_angles, step=step_size)

            # Update the robot arm with interpolated angles
            visualizer.update_angles(current_angles)
            with open('output.csv', mode='a', newline='') as file:
                writer = csv.writer(file)
                # Write the data row
                writer.writerow([1, 1, *current_angles])

            # Get the actual end-effector position from the visualizer
            e_pos = visualizer.get_end_effector_position()

            # Check if the target is reached
            error = np.linalg.norm(np.array(target_pos) - np.array(e_pos))
            if error < 0.01:  # If within 1cm of the target
                current_target_index = (current_target_index + 1) % len(target_positions)

        except ValueError as e:
            current_target_index = (current_target_index + 1) % len(target_positions)

    pygame.quit()
