# target_publisher_gui.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import pygame
import sys
import math
import json
import time
from pathlib import Path
import os
class TargetPositionGUI(Node):
    def __init__(self):
        super().__init__('target_position_gui')
        
        # ROS Publishers
        self.position_publisher = self.create_publisher(String, 'target_position', 10)
        
        # Subscribe to current angles for display
        self.angle_subscription = self.create_subscription(
            Float32MultiArray,
            'target_angles',
            self.angle_callback,
            10
        )
        
        # Initialize PyGame
        pygame.init()
        pygame.joystick.init()
        
        # Setup display
        self.width = 800
        self.height = 600
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption('Robot Arm Controller')
        
        # Colors
        self.BLACK = (0, 0, 0)
        self.WHITE = (255, 255, 255)
        self.RED = (255, 0, 0)
        self.GREEN = (0, 255, 0)
        self.BLUE = (0, 0, 255)
        self.YELLOW = (255, 255, 0)
        self.GRAY = (128, 128, 128)
        
        # Control variables
        self.target_x = 0.0
        self.target_y = 0.0
        self.step_size = 0.01  # Movement step size
        self.current_angles = [0.0, 0.0, 0.0]
        
        # Grid settings
        self.pixels_per_meter = 1000
        self.grid_size = 0.02
        
        # Recording variables
        self.recording = False
        self.recorded_positions = []
        self.playback_active = False
        self.playback_index = 0
        self.record_start_time = 0
        
        # Speed control
        self.playback_speed = 1.0  # Normal speed
        
        # Initialize controller if available
        self.joystick = None
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.get_logger().info('Controller connected')
        
        # Font for text
        self.font = pygame.font.Font(None, 36)
        
        # Create recordings directory if it doesn't exist
        self.recordings_dir = Path('recordings')
        self.recordings_dir.mkdir(exist_ok=True)
        
        self.get_logger().info('Target Position GUI Node Started')
    
    def start_recording(self):
        """Start recording positions"""
        self.recording = True
        self.recorded_positions = []
        self.record_start_time = time.time()
        self.get_logger().info('Recording started')
    
    def stop_recording(self):
        """Stop recording and save positions"""
        if self.recording:
            self.recording = False
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            
            # Get current directory
            current_directory = Path(os.getcwd())
            
            # Create file path in the current directory
            filename = current_directory / f'recording_{timestamp}.json'
            
            with open(filename, 'w') as f:
                json.dump(self.recorded_positions, f)
            
            self.get_logger().info(f'Recording saved to {filename}')
    
    def load_recording(self):
        """Load the most recent recording"""
        try:
            recordings = list(self.recordings_dir.glob('*.json'))
            if recordings:
                latest_recording = max(recordings, key=lambda x: x.stat().st_mtime)
                with open(latest_recording, 'r') as f:
                    self.recorded_positions = json.load(f)
                self.get_logger().info(f'Loaded recording from {latest_recording}')
                return True
        except Exception as e:
            self.get_logger().error(f'Error loading recording: {e}')
        return False
    
    def start_playback(self):
        """Start playing back recorded positions"""
        if self.recorded_positions:
            self.playback_active = True
            self.playback_index = 0
            self.get_logger().info('Playback started')
    
    def update_playback(self):
        """Update position during playback"""
        if self.playback_active and self.recorded_positions:
            if self.playback_index < len(self.recorded_positions):
                pos = self.recorded_positions[self.playback_index]
                self.target_x = pos['x']
                self.target_y = pos['y']
                self.playback_index += 1 * self.playback_speed
                if self.playback_index >= len(self.recorded_positions):
                    self.playback_active = False
                    self.get_logger().info('Playback completed')
    
    def angle_callback(self, msg):
        """Update current angles when received from robot controller"""
        self.current_angles = list(msg.data)
    
    def publish_target(self):
        """Publish current target position"""
        msg = String()
        msg.data = f"{self.target_x:.3f} {self.target_y:.3f}"
        self.position_publisher.publish(msg)
        
        # Record position if recording is active
        if self.recording:
            self.recorded_positions.append({
                'x': self.target_x,
                'y': self.target_y,
                'timestamp': time.time() - self.record_start_time
            })
    
    def draw_grid(self):
        """Draw coordinate grid"""
        center_x = self.width // 2
        center_y = self.height // 2
        
        for x in range(0, self.width, int(self.grid_size * self.pixels_per_meter)):
            alpha = 128 if x == center_x else 64
            color = (*self.GRAY[:3], alpha)
            pygame.draw.line(self.screen, color, (x, 0), (x, self.height))
            
        for y in range(0, self.height, int(self.grid_size * self.pixels_per_meter)):
            alpha = 128 if y == center_y else 64
            color = (*self.GRAY[:3], alpha)
            pygame.draw.line(self.screen, color, (0, y), (self.width, y))
    
    def world_to_screen(self, x, y):
        """Convert world coordinates (meters) to screen coordinates (pixels)"""
        screen_x = self.width // 2 + int(x * self.pixels_per_meter)
        screen_y = self.height // 2 - int(y * self.pixels_per_meter)
        return screen_x, screen_y
    
    def screen_to_world(self, screen_x, screen_y):
        """Convert screen coordinates (pixels) to world coordinates (meters)"""
        x = (screen_x - self.width // 2) / self.pixels_per_meter
        y = (self.height // 2 - screen_y) / self.pixels_per_meter
        return x, y
    
    def handle_ps4_controller(self):
        """Handle PS4 controller input"""
        if not self.joystick:
            return
            
        # Left stick for position control
        x_axis = self.joystick.get_axis(0)
        y_axis = self.joystick.get_axis(1)
        
        # Dead zone
        dead_zone = 0.1
        if abs(x_axis) > dead_zone:
            self.target_x += x_axis * self.step_size
        if abs(y_axis) > dead_zone:
            self.target_y -= y_axis * self.step_size
        
        # Right stick for speed control
        speed_axis = self.joystick.get_axis(3)  # Vertical axis of right stick
        if abs(speed_axis) > dead_zone:
            self.step_size = max(0.001, min(0.05, self.step_size * (1 + speed_axis * 0.1)))
        
        # Button mappings (PS4)
        try:
            # Circle (Recording control)
            if self.joystick.get_button(1):  # Circle
                if not self.recording:
                    self.start_recording()
                else:
                    self.stop_recording()
            
            # Triangle (Load and play recording)
            if self.joystick.get_button(3):  # Triangle
                if not self.playback_active:
                    if self.load_recording():
                        self.start_playback()
            
            # Square (Stop playback)
            if self.joystick.get_button(0):  # Square
                self.playback_active = False
            
            # D-pad for playback speed control
            if self.joystick.get_button(14):  # D-pad up
                self.playback_speed = min(2.0, self.playback_speed + 0.1)
            if self.joystick.get_button(15):  # D-pad down
                self.playback_speed = max(0.1, self.playback_speed - 0.1)
            
            # L1/R1 for step size adjustment
            if self.joystick.get_button(4):  # L1
                self.step_size = max(0.001, self.step_size * 0.9)
            if self.joystick.get_button(5):  # R1
                self.step_size = min(0.05, self.step_size * 1.1)
            
        except pygame.error:
            pass
    
    def run(self):
        """Main loop"""
        try:
            while rclpy.ok():
                # Handle PyGame events
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        return
                    elif event.type == pygame.MOUSEBUTTONDOWN:
                        if event.button == 1:  # Left click
                            self.target_x, self.target_y = self.screen_to_world(*event.pos)
                            self.publish_target()
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_r:
                            if not self.recording:
                                self.start_recording()
                            else:
                                self.stop_recording()
                        elif event.key == pygame.K_p:
                            if not self.playback_active:
                                if self.load_recording():
                                    self.start_playback()
                
                # Handle controller input
                self.handle_ps4_controller()
                
                # Handle keyboard input if not in playback
                if not self.playback_active:
                    keys = pygame.key.get_pressed()
                    if keys[pygame.K_LEFT]:
                        self.target_x -= self.step_size
                    if keys[pygame.K_RIGHT]:
                        self.target_x += self.step_size
                    if keys[pygame.K_UP]:
                        self.target_y += self.step_size
                    if keys[pygame.K_DOWN]:
                        self.target_y -= self.step_size
                else:
                    self.update_playback()
                
                # Clear screen
                self.screen.fill(self.BLACK)
                
                # Draw grid
                self.draw_grid()
                
                # Draw target position
                target_screen_x, target_screen_y = self.world_to_screen(self.target_x, self.target_y)
                pygame.draw.circle(self.screen, self.RED, (target_screen_x, target_screen_y), 5)
                
                # Draw text information
                status_color = self.RED if self.recording else self.WHITE
                pos_text = self.font.render(f'Target: ({self.target_x:.3f}, {self.target_y:.3f})', True, self.WHITE)
                angles_text = self.font.render(f'Angles: [{", ".join([f"{a:.1f}" for a in self.current_angles])}]', True, self.WHITE)
                status_text = self.font.render('Recording' if self.recording else 'Playback' if self.playback_active else '', True, status_color)
                speed_text = self.font.render(f'Speed: {self.playback_speed:.1f}x', True, self.WHITE)
                step_text = self.font.render(f'Step: {self.step_size:.3f}m', True, self.WHITE)
                
                self.screen.blit(pos_text, (10, 10))
                self.screen.blit(angles_text, (10, 50))
                self.screen.blit(status_text, (10, 90))
                self.screen.blit(speed_text, (10, 130))
                self.screen.blit(step_text, (10, 170))
                
                # Update display
                pygame.display.flip()
                
                # Publish target position
                self.publish_target()
                
                # Process ROS callbacks
                rclpy.spin_once(self, timeout_sec=0)
                
                # Control update rate
                pygame.time.wait(50)  # 20 Hz update rate
                
        except Exception as e:
            self.get_logger().error(f'Error in main loop: {e}')
        finally:
            pygame.quit()

def main(args=None):
    rclpy.init(args=args)
    gui = TargetPositionGUI()
    
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        gui.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()