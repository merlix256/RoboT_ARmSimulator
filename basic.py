from robot_arm_visualizer import RobotArmVisualizer
import math

joint_lengths = [120, 100, 80, 60]
visualizer = RobotArmVisualizer(joint_lengths)

# Let's move just the first joint back and forth
angle = 0
while visualizer.update():
    # First joint moves, others stay fixed
    angles = [
        45 * math.sin(angle),  # This joint will move back and forth
        30,                    # These joints stay fixed
        -30,
        0
    ]
    visualizer.update_angles(angles)
    angle += 0.02  # Control the speed of movement