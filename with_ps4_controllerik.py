import csv
import numpy as np
import math
import time
import pygame
from robot_arm_visualize_with_metric import RobotArmVisualizer

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

# Initialize the PS4 controller using pygame
def init_ps4_controller():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        raise RuntimeError("No PS4 controller connected.")
    controller = pygame.joystick.Joystick(0)
    controller.init()
    print(f"Controller initialized: {controller.get_name()}")
    return controller

# Function to clamp the target position within the arm's reach
def clamp_target_position(target_pos, joint_lengths):
    x_target, y_target = target_pos
    max_reach = sum(joint_lengths)  # Maximum reach of the arm
    distance = math.sqrt(x_target**2 + y_target**2)

    if distance > max_reach:
        # Scale the target to the maximum reachable distance
        scale_factor = max_reach / distance
        x_target *= scale_factor
        y_target *= scale_factor

    return [x_target, y_target]

# Function to save data to CSV (end effector position + joint angles)
def save_data_to_csv(file_path, end_effector_pos, joint_angles):
    with open(file_path, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(end_effector_pos + tuple(joint_angles))  # Save position + angles

# Main script
if __name__ == "__main__":
    joint_lengths = [0.1, 0.08, 0.05]  # Lengths for a 3-joint arm
    visualizer = RobotArmVisualizer(
        joint_lengths=joint_lengths,
        pixels_per_meter=1000,
        grid_size_meters=0.1,
    )

    current_angles = [0, 0, 0]  # Initial angles for the arm
    step_size = 5  # Degrees per frame for smooth motion
    controller = init_ps4_controller()

    # Starting position for the end effector
    current_target_pos = [0.0, 0.0]
    speed = 0.01  # Speed for joystick-controlled movement (meters per frame)

    # CSV file to store end effector positions and joint angles
    csv_file_path = 'robot_arm_data.csv'

    # Write the header if the file doesn't exist
    with open(csv_file_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['x_pos', 'y_pos', 'angle1', 'angle2', 'angle3'])  # CSV header

    while visualizer.update():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()

        # Read joystick values
        left_stick_x = controller.get_axis(0)  # Left joystick X-axis
        left_stick_y = controller.get_axis(1)  # Left joystick Y-axis

        # Adjust target position based on joystick input
        current_target_pos[0] += left_stick_x * speed  # Adjust x
        current_target_pos[1] -= left_stick_y * speed  # Adjust y (invert for screen coordinates)

        # Clamp the target position to ensure it's within reach
        current_target_pos = clamp_target_position(current_target_pos, joint_lengths)

        try:
            # Calculate target angles using IK
            target_angles = inverse_kinematics_3d(joint_lengths, current_target_pos)

            # Interpolate angles for smoother motion
            current_angles = interpolate_angles(current_angles, target_angles, step=step_size)

            # Update the robot arm with interpolated angles
            visualizer.update_angles(current_angles)

            # Get the actual end-effector position from the visualizer
            e_pos = visualizer.get_end_effector_position()

            # Print current state
            print(f"Joystick Target: {current_target_pos}, End effector: {e_pos}, Current angles: {current_angles}")

            # Save the end effector position and joint angles to CSV
            save_data_to_csv(csv_file_path, e_pos, current_angles)

        except ValueError as e:
            print(f"Error: {e}")
