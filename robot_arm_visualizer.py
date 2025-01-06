import pygame
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict
import math
import colorsys

@dataclass
class JointState:
    """Joint state class with visualization and physical properties"""
    angle: float
    length: float
    color: Tuple[int, int, int] = (100, 100, 255)
    min_angle: float = -180.0
    max_angle: float = 180.0
    max_torque: float = 100.0  # Newton-meters
    name: str = "Joint"
    
    def to_dict(self) -> Dict:
        """Convert joint state to dictionary for easy access"""
        return {
            "angle": round(self.angle, 2),
            "length": round(self.length, 2),
            "min_angle": self.min_angle,
            "max_angle": self.max_angle,
            "max_torque": self.max_torque,
            "name": self.name
        }

class RobotArmVisualizer:
    """Enhanced educational robot arm visualizer"""
    def __init__(self, joint_lengths: List[float], width: int = 1200, height: int = 800):
        pygame.init()
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Educational Robot Arm Visualizer")
        # Add direction indicator properties
        self.indicator_radius = 40
        self.indicator_center = (80, height - 80)  # Position in bottom-left corner
        self.indicator_dot_radius = 6

        # Setup
        self.base_position = (width // 3, height // 2)
        self.joints = [
            JointState(
                angle=0.0,
                length=length,
                color=self._generate_color(i),
                min_angle=-180,
                max_angle=180,
                max_torque=100 - i * 15,  # Decreasing torque for each joint
                name=f"Joint {i+1}"
            ) for i, length in enumerate(joint_lengths)
        ]
        
        # Visualization options
        self.show_trail = True
        self.trail_points = []
        self.max_trail_points = 100
        self.show_workspace = True
        self.show_grid = True
        self.grid_size = 60
        self.selected_joint: Optional[int] = None
        
        # UI setup
        pygame.font.init()
        self.font = pygame.font.Font(None, 24)
        self.large_font = pygame.font.Font(None, 36)
        
        self.running = True
        self.clock = pygame.time.Clock()
    def _draw_direction_indicator(self):
        """Draw a direction indicator in the bottom-left corner"""
        # Draw the main circle
        pygame.draw.circle(self.screen, (150, 150, 150), 
                         self.indicator_center, 
                         self.indicator_radius, 2)
        
        # Draw N/S/E/W marks
        marks = [(0, -1), (1, 0), (0, 1), (-1, 0)]  # North, East, South, West
        for dx, dy in marks:
            start_pos = (
                self.indicator_center[0] + dx * (self.indicator_radius - 10),
                self.indicator_center[1] + dy * (self.indicator_radius - 10)
            )
            end_pos = (
                self.indicator_center[0] + dx * self.indicator_radius,
                self.indicator_center[1] + dy * self.indicator_radius
            )
            pygame.draw.line(self.screen, (150, 150, 150), start_pos, end_pos, 2)

        # Draw the direction dot based on base joint angle
        base_angle = self.joints[0].angle
        dot_x = self.indicator_center[0] + math.cos(math.radians(base_angle - 90)) * self.indicator_radius
        dot_y = self.indicator_center[1] + math.sin(math.radians(base_angle - 90)) * self.indicator_radius
        
        # Draw dot
        pygame.draw.circle(self.screen, (255, 100, 100),
                         (int(dot_x), int(dot_y)),
                         self.indicator_dot_radius)
        
        # Draw center dot
        pygame.draw.circle(self.screen, (150, 150, 150),
                         self.indicator_center, 3)
        
        # Add text label
        label = self.font.render("Base Direction", True, (200, 200, 200))
        self.screen.blit(label, (self.indicator_center[0] - 50, 
                                self.indicator_center[1] + self.indicator_radius + 10))   
    def _generate_color(self, index: int) -> Tuple[int, int, int]:
        """Generate visually pleasing color using golden ratio"""
        hue = (index * 0.618033988749895) % 1.0
        rgb = colorsys.hsv_to_rgb(hue, 0.8, 0.95)
        return tuple(int(255 * x) for x in rgb)
        
    def _draw_grid(self) -> None:
        """Draw coordinate grid"""
        for x in range(0, self.width, self.grid_size):
            alpha = 255 if x % (self.grid_size * 2) == 0 else 100
            color = (50, 50, 50, alpha)
            pygame.draw.line(self.screen, color, (x, 0), (x, self.height), 1)
            
        for y in range(0, self.height, self.grid_size):
            alpha = 255 if y % (self.grid_size * 2) == 0 else 100
            color = (50, 50, 50, alpha)
            pygame.draw.line(self.screen, color, (0, y), (self.width, y), 1)
            
    def _draw_joint_info(self, joint_idx: int, position: Tuple[float, float]) -> None:
        """Draw detailed joint information"""
        joint = self.joints[joint_idx]
        info = joint.to_dict()
        
        # Create info box
        box_width = 250
        box_height = 150
        box_x = self.width - box_width - 10
        box_y = 10
        
        # Draw semi-transparent background
        s = pygame.Surface((box_width, box_height))
        s.set_alpha(128)
        s.fill((30, 30, 30))
        self.screen.blit(s, (box_x, box_y))
        
        # Draw joint information
        texts = [
            f"{info['name']}:",
            f"Angle: {info['angle']}°",
            f"Length: {info['length']} units",
            f"Position: ({int(position[0])}, {int(position[1])})",
            f"Torque Limit: {info['max_torque']} Nm",
            f"Angle Limits: [{info['min_angle']}°, {info['max_angle']}°]"
        ]
        
        for i, text in enumerate(texts):
            surface = self.font.render(text, True, (255, 255, 255))
            self.screen.blit(surface, (box_x + 10, box_y + 10 + i * 25))
            
    def _is_point_near_joint(self, point: Tuple[float, float], joint_pos: Tuple[float, float], 
                            threshold: int = 15) -> bool:
        """Check if a point is near a joint position"""
        return math.sqrt((point[0] - joint_pos[0])**2 + (point[1] - joint_pos[1])**2) < threshold
        
    def get_joint_positions(self) -> List[Tuple[float, float]]:
        """Get current positions of all joints - useful for external control"""
        return self.calculate_positions()
        
    def get_end_effector_position(self) -> Tuple[float, float]:
        """Get current end effector position - useful for external control"""
        return self.calculate_positions()[-1]
        
    def get_joint_states(self) -> List[Dict]:
        """Get current state of all joints - useful for external control"""
        return [joint.to_dict() for joint in self.joints]

    def _draw_gradient_line(self, start: Tuple[float, float], end: Tuple[float, float], 
                           color: Tuple[int, int, int]) -> None:
        """Draw a line with gradient effect"""
        steps = 10
        for i in range(steps):
            t = i / steps
            start_pos = (start[0] + (end[0] - start[0]) * t,
                        start[1] + (end[1] - start[1]) * t)
            end_pos = (start[0] + (end[0] - start[0]) * (t + 1/steps),
                      start[1] + (end[1] - start[1]) * (t + 1/steps))
            
            alpha = 255 - int(155 * (i / steps))
            current_color = tuple(int(c * alpha/255) for c in color)
            pygame.draw.line(self.screen, current_color, start_pos, end_pos, 5)
            
    def update_angles(self, angles: List[float]) -> None:
        """Update joint angles from external source"""
        if len(angles) != len(self.joints):
            raise ValueError("Number of angles must match number of joints")
        for joint, angle in zip(self.joints, angles):
            joint.angle = np.clip(angle, joint.min_angle, joint.max_angle)
            
    def calculate_positions(self) -> List[Tuple[float, float]]:
        """Calculate all joint positions"""
        positions = [self.base_position]
        current_point = self.base_position
        cumulative_angle = 0
        
        for joint in self.joints:
            cumulative_angle += joint.angle
            angle_rad = math.radians(cumulative_angle)
            next_point = (
                current_point[0] + joint.length * math.cos(angle_rad),
                current_point[1] + joint.length * math.sin(angle_rad)
            )
            positions.append(next_point)
            current_point = next_point
            
        return positions
        
    def draw(self) -> None:
        """Draw the enhanced robot arm visualization"""
        self.screen.fill((20, 20, 20))  # Dark background
        
        # Draw grid if enabled
        if self.show_grid:
            self._draw_grid()
        
        # Draw workspace boundary
        if self.show_workspace:
            total_length = sum(joint.length for joint in self.joints)
            pygame.draw.circle(self.screen, (30, 30, 30), self.base_position, total_length, 1)
        
        # Draw trail
        if self.show_trail and len(self.trail_points) > 1:
            pygame.draw.lines(self.screen, (100, 100, 100), False, self.trail_points, 2)
            
        # Get current positions
        positions = self.calculate_positions()
        
        # Draw base
        pygame.draw.circle(self.screen, (150, 150, 150), self.base_position, 12)
        
        # Draw arm segments and joints
        for i in range(len(positions) - 1):
            start = positions[i]
            end = positions[i + 1]
            
            # Draw joint constraints
            min_angle = math.radians(sum(j.angle for j in self.joints[:i]) + self.joints[i].min_angle)
            max_angle = math.radians(sum(j.angle for j in self.joints[:i]) + self.joints[i].max_angle)
            pygame.draw.arc(self.screen, (50, 50, 50),
                          (start[0] - 20, start[1] - 20, 40, 40),
                          min_angle, max_angle, 2)
            
            # Draw segment with gradient
            self._draw_gradient_line(start, end, self.joints[i].color)
            
            # Draw joint
            joint_color = (200, 200, 200) if i == self.selected_joint else (150, 150, 150)
            pygame.draw.circle(self.screen, joint_color,
                             (int(start[0]), int(start[1])), 8)
        
        # Draw end effector
        end_pos = positions[-1]
        pygame.draw.circle(self.screen, (255, 100, 100),
                         (int(end_pos[0]), int(end_pos[1])), 6)
        
        # Update trail
        if self.show_trail:
            self.trail_points.append((int(end_pos[0]), int(end_pos[1])))
            if len(self.trail_points) > self.max_trail_points:
                self.trail_points.pop(0)
                
        # Draw info
        self._draw_info(positions[-1])
        
        # Draw selected joint info
        if self.selected_joint is not None:
            self._draw_joint_info(self.selected_joint, positions[self.selected_joint])
        # Add direction indicator
        self._draw_direction_indicator()
        
        pygame.display.flip()
        
    def _draw_info(self, end_pos: Tuple[float, float]) -> None:
        """Draw information overlay"""
        info_texts = [
            f"End Effector: ({int(end_pos[0])}, {int(end_pos[1])})",
            "Controls:",
            "T: Toggle trail",
            "W: Toggle workspace",
            "G: Toggle grid",
            "Click joints for info",
            "ESC: Quit"
        ]
        
        for i, text in enumerate(info_texts):
            surface = self.font.render(text, True, (255, 255, 255))
            self.screen.blit(surface, (10, 10 + i * 25))
            
    def check_events(self) -> bool:
        """Check for quit and control events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return False
                elif event.key == pygame.K_t:
                    self.show_trail = not self.show_trail
                    self.trail_points.clear()
                elif event.key == pygame.K_w:
                    self.show_workspace = not self.show_workspace
                elif event.key == pygame.K_g:
                    self.show_grid = not self.show_grid
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mouse_pos = pygame.mouse.get_pos()
                positions = self.calculate_positions()
                
                # Check if clicked near any joint
                self.selected_joint = None
                for i, pos in enumerate(positions[:-1]):  # Exclude end effector
                    if self._is_point_near_joint(mouse_pos, pos):
                        self.selected_joint = i
                        break
        return True
        
    def update(self) -> bool:
        """Update visualization, return False if should quit"""
        self.running = self.check_events()
        self.draw()
        self.clock.tick(60)
        return self.running

# Example usage showing how to program the robot:
if __name__ == "__main__":
    # Create a robot arm with 4 joints of different lengths
    joint_lengths = [120, 100, 80, 60]  # Length of each arm segment
    visualizer = RobotArmVisualizer(joint_lengths)
    
    # Example of different ways to control the robot:
    
    # 1. Simple animation
    angle = 0
    while visualizer.update():
        # Animate joints with sinusoidal motion
        angles = [
            45 * math.sin(angle),
            45 * math.cos(angle),
            30 * math.sin(angle * 2),
            30 * math.cos(angle * 2)
        ]
        visualizer.update_angles(angles)
        angle += 0.02
        
        # Example of getting robot state
        end_effector = visualizer.get_end_effector_position()
        joint_positions = visualizer.get_joint_positions()
        joint_states = visualizer.get_joint_states()

        print(end_effector)
        
    pygame.quit()

# Other example control patterns:

"""
# 2. Position-based control
def move_to_position(target_x: float, target_y: float):
    while visualizer.update():
        current_pos = visualizer.get_end_effector_position()
        # Implement inverse kinematics here
        # Update angles based on target position
        
# 3. Sequential movement
def execute_sequence():
    movements = [
        [0, 45, 0, 0],
        [45, 0, 45, 0],
        [0, -45, 0, 45]
    ]
    for angles in movements:
        visualizer.update_angles(angles)
        # Add delay or wait for completion
        
# 4. Interactive control
def keyboard_control():
    while visualizer.update():
        keys = pygame.key.get_pressed()
        # Implement keyboard-based joint control
"""