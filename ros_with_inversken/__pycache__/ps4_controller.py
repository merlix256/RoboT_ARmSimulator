# robot_controller.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from inverse_kinematics import InverseKinematics3D, interpolate_angles
from robot_arm_visualize_with_metric import RobotArmVisualizer
import time

class RobotArmController(Node):
    def __init__(self):
        super().__init__('robot_arm_controller')
        self.subscription = self.create_subscription(
            String,
            'target_position',
            self.target_callback,
            10)
        
        # Initialize robot arm components
        self.joint_lengths = [0.1, 0.08, 0.05]
        self.visualizer = RobotArmVisualizer(
            joint_lengths=self.joint_lengths,
            pixels_per_meter=1000,
            grid_size_meters=0.1,
        )
        self.ik_solver = InverseKinematics3D(self.joint_lengths)
        self.current_angles = [0, 0, 0]
        self.step_size = 2  # Degrees per update
        self.target_angles = None
        
        # Create a timer for visualization updates
        self.create_timer(0.005, self.update_visualization)
        
        self.get_logger().info('Robot Arm Controller Node Started')

    def target_callback(self, msg):
        try:
            # Parse target position from message
            x, y = map(float, msg.data.split())
            target_position = (x, y)
            
            # Calculate new target angles
            self.target_angles = self.ik_solver.calculate(target_position)
            self.get_logger().info(f'New target received: {target_position}')
            self.get_logger().info(f'Target angles: {self.target_angles}')
            
        except ValueError as e:
            self.get_logger().error(f'Error processing target position: {e}')

    def update_visualization(self):
        if self.target_angles is None:
            return
            
        if self.visualizer.update():
            # Interpolate angles for smoother motion
            self.current_angles = interpolate_angles(
                self.current_angles, 
                self.target_angles, 
                step=self.step_size
            )

            # Update the visualizer with the interpolated angles
            self.visualizer.update_angles(self.current_angles)

            # Get and display current end-effector position
            e_pos = self.visualizer.get_end_effector_position()
            self.get_logger().debug(f'Current End-Effector Position: {e_pos}')

def main(args=None):
    rclpy.init(args=args)
    controller = RobotArmController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
        
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()