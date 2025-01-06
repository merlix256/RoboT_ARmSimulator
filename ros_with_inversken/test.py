from inverse_kinematics import InverseKinematics3D, interpolate_angles
from robot_arm_visualize_with_metric import RobotArmVisualizer
import time
# Joint lengths and visualization setup
joint_lengths = [0.1, 0.08, 0.05]  # Lengths for a 3-joint arm
visualizer = RobotArmVisualizer(
        joint_lengths=joint_lengths,
        pixels_per_meter=1000,
        grid_size_meters=0.1,
    )

# Create an instance of the IK solver
ik_solver = InverseKinematics3D(joint_lengths)

# Define the target position
target_position = (0.05, -0.07)

# Initial joint angles (all set to 0 degrees initially)
current_angles = [0, 0, 0]

# Step size for smooth interpolation
step_size = 2  # Degrees per update

try:
    # Calculate target angles using the IK solver
    target_angles = ik_solver.calculate(target_position)
    print(f"Target Angles: {target_angles}")

    # Run the visualizer loop
    while visualizer.update():
        # Interpolate angles for smoother motion
        current_angles = interpolate_angles(current_angles, target_angles, step=step_size)

        # Update the visualizer with the interpolated angles
        visualizer.update_angles(current_angles)

        # Get the current end-effector position
        e_pos = visualizer.get_end_effector_position()

        # Display the current end-effector position
        print(f"Current End-Effector Position: {e_pos}")

        # Check if the target position is reached
        
        error = ik_solver.calculate_error(e_pos, target_position)
        
        if error < 0.01:  # Within 1 cm of the target
            print("End-effector reached the target position!")


        time.sleep(0.005)  # Add delay to simulate motion time
        
except ValueError as e:
    print(f"Error: {e}")


