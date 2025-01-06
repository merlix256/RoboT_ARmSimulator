import csv
import pygame
import math
from robot_arm_visualize_with_metric import RobotArmVisualizer

# Joint lengths and visualization setup
joint_lengths = [0.1, 0.08, 0.05]  # Lengths for a 3-joint arm
visualizer = RobotArmVisualizer(
        joint_lengths=joint_lengths,
        pixels_per_meter=1000,
        grid_size_meters=0.1,
    )


# Read angles from CSV file
angles_data = []
with open('output.csv', 'r') as file:
    reader = csv.DictReader(file)
    for row in reader:
        angle1 = float(row['angle1'])
        angle2 = float(row['angle2'])
        angle3 = float(row['angle3'])
        # If there are fewer angles than joints, fill in the remaining angles as 0
        angles = [angle1, angle2, angle3] + [0] * (len(joint_lengths) - 3)
        angles_data.append(angles)

# Simple animation loop
angle_index = 0
while visualizer.update():
    if angle_index < len(angles_data):
        # Get the angles from the CSV data
        angles = angles_data[angle_index]
        visualizer.update_angles(angles)  # Update robot arm's angles
        angle_index += 1  # Increment to the next set of angles
    
    # Optional: Print the end effector position for debugging
    e_pos = visualizer.get_end_effector_position()  # Get end effector position in meters
    print(f"End effector position: {e_pos}")
    
    # Sleep to control the speed of the animation
    pygame.time.wait(10)  # Wait for 100ms between frames

pygame.quit()
