import logging
from robot_controller import RobotController
import time

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

def main():
    # Initialize robot
    joint_lengths = [0.1, 0.08, 0.05]
    robot = RobotController(
        joint_lengths=joint_lengths,
        arduino_port='COM5',  # Change to your port
        simulation=True  # Set to True to run with visualization
    )

    # Define waypoints
    waypoints = [
        (0.1, 0.1),
        (0.08, 0.15),
        (-0.05, 0.1),
        
    ]

    try:
        current_point = 0
        while robot.update_visualization():
            target = waypoints[current_point]
            
            if robot.move_to_position(target):
                print(f"Reached target {current_point + 1}")
                time.sleep(0.5)
                current_point = (current_point + 1) % len(waypoints)
            else:
                print(f"Failed to reach target {current_point + 1}")
                
    except KeyboardInterrupt:
        print("\nStopping robot...")
    finally:
        robot.close()

if __name__ == "__main__":
    main()