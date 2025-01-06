from robot_arm_visualizer import RobotArmVisualizer
import numpy as np
import math

# Initialize the visualizer with joint lengths
joint_lengths = [120, 100, 80, 60]
visualizer = RobotArmVisualizer(joint_lengths)

def calculate_forward_kinematics(angles, lengths):
    """Calculate the forward kinematics for the given joint angles."""
    x, y = 0, 0
    cumulative_angle = 0
    for i in range(len(angles)):
        cumulative_angle += angles[i]
        x += lengths[i] * math.cos(cumulative_angle)
        y += lengths[i] * math.sin(cumulative_angle)
    return np.array([x, y])

def calculate_jacobian(angles, lengths):
    """Calculate the Jacobian matrix for the robotic arm."""
    n = len(angles)
    J = np.zeros((2, n))
    
    for i in range(n):
        cumulative_angle = sum(angles[:i+1])
        J[0, i] = -sum(lengths[j] * math.sin(sum(angles[:j+1])) for j in range(i, n))
        J[1, i] = sum(lengths[j] * math.cos(sum(angles[:j+1])) for j in range(i, n))
        
    return J

def inverse_kinematics(target_pos, initial_angles, lengths, alpha=0.01, tolerance=1e-3, max_iterations=1000):
    """Perform inverse kinematics to reach the target position."""
    angles = np.array(initial_angles, dtype=np.float64)
    for iteration in range(max_iterations):
        current_pos = calculate_forward_kinematics(angles, lengths)
        error = target_pos - current_pos
        
        if np.linalg.norm(error) < tolerance:
            print(f"Converged in {iteration} iterations.")
            return angles
        
        J = calculate_jacobian(angles, lengths)
        J_pseudo_inverse = np.linalg.pinv(J)
        
        delta_angles = J_pseudo_inverse @ error
        angles += delta_angles * alpha
        
        visualizer.update_angles(angles)
        
    print("Did not converge.")
    return angles

# Example target position to reach
target_position = [150, 100]

# Initial guess for joint angles
initial_angles = [0, 0, 0, 0]

# Perform inverse kinematics to reach the target position
final_angles = inverse_kinematics(np.array(target_position), initial_angles, joint_lengths)

print("Final Joint Angles:", final_angles)

# Verify final position
final_position = calculate_forward_kinematics(final_angles, joint_lengths)
print(f"Target Position: {target_position}")
print(f"Final End Effector Position: {final_position}")

# Keep the visualizer running to observe the result
while visualizer.update():
    pass
