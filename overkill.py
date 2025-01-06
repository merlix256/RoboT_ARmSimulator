import pygame
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict
import math
import colorsys
from enum import Enum
import json

class VisualizationMode(Enum):
    """Different visualization modes for the robot arm"""
    NORMAL = "Normal"
    WIREFRAME = "Wireframe"
    THERMAL = "Thermal"
    GHOST = "Ghost"

@dataclass
class JointState:
    """Enhanced joint state class with additional properties"""
    angle: float
    length: float
    color: Tuple[int, int, int] = (100, 100, 255)
    min_angle: float = -180.0
    max_angle: float = 180.0
    velocity: float = 0.0
    acceleration: float = 0.0
    torque: float = 0.0
    name: str = ""

class InverseKinematicsSolver:
    def __init__(self, joint_lengths: List[float]):
        self.joint_lengths = joint_lengths
        
    def solve_FABRIK(self, target: Tuple[float, float], base_pos: Tuple[float, float], 
                     current_angles: List[float]) -> List[float]:
        """Forward And Backward Reaching Inverse Kinematics solver"""
        num_joints = len(self.joint_lengths)
        tolerance = 1.0
        max_iterations = 10
        
        # Initialize joint positions
        positions = []
        current_pos = base_pos
        total_angle = 0
        for i, length in enumerate(self.joint_lengths):
            total_angle += current_angles[i]
            angle_rad = math.radians(total_angle)
            current_pos = (
                current_pos[0] + length * math.cos(angle_rad),
                current_pos[1] + length * math.sin(angle_rad)
            )
            positions.append(current_pos)
            
        for _ in range(max_iterations):
            # Forward reaching
            positions[-1] = target
            for i in range(num_joints - 2, -1, -1):
                direction = np.array(positions[i]) - np.array(positions[i + 1])
                direction = direction / np.linalg.norm(direction)
                positions[i] = positions[i + 1] + direction * self.joint_lengths[i]
                
            # Backward reaching
            positions[0] = base_pos
            for i in range(num_joints - 1):
                direction = np.array(positions[i + 1]) - np.array(positions[i])
                direction = direction / np.linalg.norm(direction)
                positions[i + 1] = positions[i] + direction * self.joint_lengths[i]
                
            # Check if target is reached
            if np.linalg.norm(np.array(positions[-1]) - np.array(target)) < tolerance:
                break
                
        # Convert positions to angles
        angles = []
        prev_angle = 0
        for i in range(len(positions) - 1):
            dx = positions[i + 1][0] - positions[i][0]
            dy = positions[i + 1][1] - positions[i][1]
            angle = math.degrees(math.atan2(dy, dx))
            if i == 0:
                angles.append(angle)
                prev_angle = angle
            else:
                relative_angle = angle - prev_angle
                angles.append(relative_angle)
                prev_angle = angle
                
        return angles


class ParticleEffect:
    """Particle system for visual effects"""
    def __init__(self):
        self.particles: List[Dict] = []
        
    def emit(self, pos: Tuple[float, float], color: Tuple[int, int, int], count: int = 5):
        """Emit new particles at the given position"""
        for _ in range(count):
            angle = random.random() * 2 * math.pi
            speed = random.uniform(1, 3)
            self.particles.append({
                'pos': list(pos),
                'vel': [math.cos(angle) * speed, math.sin(angle) * speed],
                'color': color,
                'life': 1.0
            })
            
    def update(self):
        """Update particle positions and lifetimes"""
        for particle in self.particles[:]:
            particle['pos'][0] += particle['vel'][0]
            particle['pos'][1] += particle['vel'][1]
            particle['life'] -= 0.02
            if particle['life'] <= 0:
                self.particles.remove(particle)
                
    def draw(self, screen):
        """Draw all active particles"""
        for particle in self.particles:
            alpha = int(255 * particle['life'])
            color = (*particle['color'], alpha)
            pos = tuple(map(int, particle['pos']))
            surface = pygame.Surface((4, 4), pygame.SRCALPHA)
            pygame.draw.circle(surface, color, (2, 2), 2)
            screen.blit(surface, (pos[0] - 2, pos[1] - 2))

class RobotArmVisualizer:
    """Enhanced robot arm visualizer with advanced features"""
    def __init__(self, joint_lengths: List[float], width: int = 1200, height: int = 800):
        pygame.init()
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Advanced Robot Arm Visualizer")
        
        # Setup
        self.base_position = (width // 3, height // 2)
        self.joints = [
            JointState(
                angle=0.0,
                length=length,
                color=self._generate_color(i),
                min_angle=-180,
                max_angle=180,
                name=f"Joint {i+1}"
            ) for i, length in enumerate(joint_lengths)
        ]
    
        # Enhanced features
        self.ik_solver = InverseKinematicsSolver(joint_lengths)
        self.particles = ParticleEffect()
        self.visualization_mode = VisualizationMode.NORMAL
        self.target_position: Optional[Tuple[float, float]] = None
        self.selected_joint: Optional[int] = None
        self.recording = False
        self.recorded_frames: List[List[float]] = []
        self.playback_mode = False
        self.playback_frame = 0
        
        # Advanced visualization options
        self.show_trail = True
        self.trail_points = []
        self.max_trail_points = 200
        self.show_workspace = True
        self.show_velocity_vectors = True
        self.show_joint_limits = True
        self.show_grid = True
        self.show_coordinates = True
        
        # UI elements
        pygame.font.init()
        self.font = pygame.font.Font(None, 24)
        self.large_font = pygame.font.Font(None, 36)
        self.ui_elements = self._create_ui_elements()
        
        # Performance monitoring
        self.fps_history = []
        self.running = True
        self.clock = pygame.time.Clock()
    def _generate_color(self, index: int) -> Tuple[int, int, int]:
            """Generate visually pleasing colors using golden ratio"""
            golden_ratio = 0.618033988749895
            hue = (index * golden_ratio) % 1.0
            # Use higher saturation and value for more vibrant colors
            saturation = 0.85
            value = 0.95
            rgb = colorsys.hsv_to_rgb(hue, saturation, value)
            # Convert to 8-bit RGB values
            return tuple(int(255 * x) for x in rgb)
    def calculate_positions(self) -> List[Tuple[float, float]]:
        """Calculate all joint positions based on current angles"""
        positions = [self.base_position]
        current_point = self.base_position
        cumulative_angle = 0
        
        for joint in self.joints:
            cumulative_angle += joint.angle
            angle_rad = math.radians(cumulative_angle)
            
            # Calculate next joint position using polar coordinates
            next_point = (
                current_point[0] + joint.length * math.cos(angle_rad),
                current_point[1] + joint.length * math.sin(angle_rad)
            )
            
            positions.append(next_point)
            current_point = next_point
            
        return positions
    def _create_ui_elements(self) -> Dict:
        """Create UI elements for the visualizer"""
        return {
            'mode_button': pygame.Rect(10, 10, 150, 30),
            'record_button': pygame.Rect(10, 50, 150, 30),
            'save_button': pygame.Rect(10, 90, 150, 30),
            'load_button': pygame.Rect(10, 130, 150, 30),
            'joint_info': pygame.Rect(self.width - 200, 10, 190, 200)
        }
    def update_angles(self, angles: List[float]) -> None:
        """Update joint angles with validation"""
        if len(angles) != len(self.joints):
            raise ValueError(f"Expected {len(self.joints)} angles, got {len(angles)}")
            
        for joint, new_angle in zip(self.joints, angles):
            # Store previous angle for velocity calculation
            prev_angle = joint.angle
            
            # Clamp angle within joint limits
            clamped_angle = np.clip(new_angle, joint.min_angle, joint.max_angle)
            
            # Update joint state
            joint.angle = clamped_angle
            
            # Calculate velocity (degrees per second)
            joint.velocity = (clamped_angle - prev_angle) * 60
            
            # Calculate torque (simplified model)
            joint.torque = joint.length * math.sin(math.radians(clamped_angle)) * 9.81
            
        # Record frame if in recording mode
        if hasattr(self, 'recording') and self.recording:
            self.recorded_frames.append([j.angle for j in self.joints])    
    def _draw_ui(self):
        """Draw all UI elements"""
        # Mode button
        pygame.draw.rect(self.screen, (60, 60, 60), self.ui_elements['mode_button'])
        text = self.font.render(f"Mode: {self.visualization_mode.value}", True, (255, 255, 255))
        self.screen.blit(text, (20, 15))
        
        # Record button
        color = (200, 60, 60) if self.recording else (60, 60, 60)
        pygame.draw.rect(self.screen, color, self.ui_elements['record_button'])
        text = self.font.render("Record" if not self.recording else "Stop", True, (255, 255, 255))
        self.screen.blit(text, (20, 55))
        
        # Joint info panel
        if self.selected_joint is not None:
            joint = self.joints[self.selected_joint]
            panel = self.ui_elements['joint_info']
            pygame.draw.rect(self.screen, (40, 40, 40), panel)
            info_texts = [
                f"Joint {self.selected_joint + 1}",
                f"Angle: {joint.angle:.1f}°",
                f"Velocity: {joint.velocity:.1f}°/s",
                f"Torque: {joint.torque:.1f} Nm",
                f"Length: {joint.length}",
                f"Limits: [{joint.min_angle}, {joint.max_angle}]"
            ]
            for i, text in enumerate(info_texts):
                surface = self.font.render(text, True, (255, 255, 255))
                self.screen.blit(surface, (panel.x + 10, panel.y + 10 + i * 25))
                
    def _draw_grid(self):
        """Draw coordinate grid"""
        grid_size = 50
        for x in range(0, self.width, grid_size):
            pygame.draw.line(self.screen, (40, 40, 40), (x, 0), (x, self.height))
        for y in range(0, self.height, grid_size):
            pygame.draw.line(self.screen, (40, 40, 40), (0, y), (self.width, y))
            
    def _apply_thermal_effect(self, surface):
        """Apply thermal vision effect to the surface"""
        thermal = pygame.surfarray.pixels3d(surface).copy()
        intensity = np.sum(thermal, axis=2) / 3
        thermal_colors = np.zeros_like(thermal)
        thermal_colors[..., 0] = np.clip(intensity * 2, 0, 255)  # Red channel
        thermal_colors[..., 2] = np.clip(255 - intensity, 0, 255)  # Blue channel
        pygame.surfarray.blit_array(surface, thermal_colors)
        
    def save_state(self, filename: str):
        """Save current robot state to file"""
        state = {
            'joints': [(j.angle, j.length, j.min_angle, j.max_angle) for j in self.joints],
            'recorded_frames': self.recorded_frames
        }
        with open(filename, 'w') as f:
            json.dump(state, f)
            
    def load_state(self, filename: str):
        """Load robot state from file"""
        with open(filename, 'r') as f:
            state = json.load(f)
            for joint, (angle, length, min_angle, max_angle) in zip(self.joints, state['joints']):
                joint.angle = angle
                joint.length = length
                joint.min_angle = min_angle
                joint.max_angle = max_angle
            self.recorded_frames = state['recorded_frames']
            
    def handle_mouse_interaction(self, pos: Tuple[int, int]):
        """Handle mouse interaction with the robot arm"""
        if self.target_position:
            # Update IK target
            self.target_position = pos
            new_angles = self.ik_solver.solve_FABRIK(pos, self.base_position, 
                                                   [j.angle for j in self.joints])
            self.update_angles(new_angles)
        else:
            # Check joint selection
            positions = self.calculate_positions()
            for i, position in enumerate(positions):
                if math.dist(position, pos) < 15:
                    self.selected_joint = i
                    break
                    
    def update(self) -> bool:
        """Update visualization state"""
        mouse_pos = pygame.mouse.get_pos()
        mouse_buttons = pygame.mouse.get_pressed()
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                # Handle UI interaction
                for element_name, rect in self.ui_elements.items():
                    if rect.collidepoint(mouse_pos):
                        if element_name == 'mode_button':
                            modes = list(VisualizationMode)
                            current_idx = modes.index(self.visualization_mode)
                            self.visualization_mode = modes[(current_idx + 1) % len(modes)]
                        elif element_name == 'record_button':
                            self.recording = not self.recording
                            if not self.recording and self.recorded_frames:
                                self.save_state("recording.json")
                        break
                else:
                    self.handle_mouse_interaction(mouse_pos)
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return False
                elif event.key == pygame.K_t:
                    self.show_trail = not self.show_trail
                elif event.key == pygame.K_g:
                    self.show_grid = not self.show_grid
                elif event.key == pygame.K_SPACE:
                    self.target_position = None
                    
        # Update particles
        self.particles.update()
        
        # Record frame if recording
        if self.recording:
            self.recorded_frames.append([j.angle for j in self.joints])
            
        # Update FPS tracking
        self.fps_history.append(self.clock.get_fps())
        if len(self.fps_history) > 100:
            self.fps_history.pop(0)
            
        self.draw()
        self.clock.tick(60)
        return True
        
    def draw(self):
        """Draw the enhanced visualization"""
        self.screen.fill((20, 20, 20))
        
        if self.show_grid:
            self._draw_grid()
            
        # Draw workspace and constraints
        if self.show_workspace:
            total_length = sum(j.length for j in self.joints)
            pygame.draw.circle(self.screen, (30, 30, 30), self.base_position, total_length, 1)
            
            if self.show_joint_limits:
                positions = self.calculate_positions()
                for i, (start, end) in enumerate(zip(positions, positions[1:])):
                    if self.visualization_mode != VisualizationMode.WIREFRAME:
                        # Draw joint constraints
                        min_angle = math.radians(sum(j.angle for j in self.joints[:i]) + 
                                               self.joints[i].min_angle)
                        max_angle = math.radians(sum(j.angle for j in self.joints[:i]) + 
                                               self.joints[i].max_angle)
                        pygame.draw.arc(self.screen, (50, 50, 50),
                                      (start[0] - 20, start[1] - 20, 40, 40),
                                      min_angle, max_angle, 2)
        
        # Draw trail
        if self.show_trail and len(self.trail_points) > 1:
            if self.visualization_mode == VisualizationMode.THERMAL:
                color = (200, 100, 50)
            else:
                color = (100, 100, 100)
            pygame.draw.lines(self.screen, color, False, self.trail_points, 2)
            
        # Get current positions
        positions = self.calculate_positions()
        
        # Draw target position if active
        if self.target_position:
            pygame.draw.circle(self.screen, (200, 50, 50), self.target_position, 8, 2)
            pygame.draw.line(self.screen, (200, 50, 50),
                           (self.target_position[0] - 10, self.target_position[1]),
                           (self.target_position[0] + 10, self.target_position[1]), 2)
            pygame.draw.line(self.screen, (200, 50, 50),
                           (self.target_position[0], self.target_position[1] - 10),
                           (self.target_position[0], self.target_position[1] + 10), 2)
            
        # Draw base
        pygame.draw.circle(self.screen, (150, 150, 150), self.base_position, 12)
        
        # Draw arm segments and joints based on visualization mode
        for i in range(len(positions) - 1):
            start = positions[i]
            end = positions[i + 1]
            
            if self.visualization_mode == VisualizationMode.WIREFRAME:
                # Simple wireframe rendering
                pygame.draw.line(self.screen, self.joints[i].color, start, end, 2)
                pygame.draw.circle(self.screen, (150, 150, 150), 
                                 (int(start[0]), int(start[1])), 4)
            elif self.visualization_mode == VisualizationMode.GHOST:
                # Ghost effect with transparency
                surface = pygame.Surface((self.width, self.height), pygame.SRCALPHA)
                color = (*self.joints[i].color, 128)  # Add alpha channel
                pygame.draw.line(surface, color, start, end, 8)
                self.screen.blit(surface, (0, 0))
            elif self.visualization_mode == VisualizationMode.THERMAL:
                # Thermal vision effect based on joint velocity
                intensity = min(255, abs(self.joints[i].velocity) * 5)
                color = (intensity, intensity//2, 0)
                self._draw_gradient_line(start, end, color)
            else:
                # Normal mode with gradient effect
                self._draw_gradient_line(start, end, self.joints[i].color)
            
            # Draw joint
            joint_color = (200, 200, 200) if i == self.selected_joint else (150, 150, 150)
            pygame.draw.circle(self.screen, joint_color,
                             (int(start[0]), int(start[1])), 8)
            
            # Draw velocity vectors if enabled
            if self.show_velocity_vectors and self.joints[i].velocity != 0:
                velocity_dir = math.radians(sum(j.angle for j in self.joints[:i+1]))
                vel_end = (
                    start[0] + math.cos(velocity_dir) * self.joints[i].velocity,
                    start[1] + math.sin(velocity_dir) * self.joints[i].velocity
                )
                pygame.draw.line(self.screen, (50, 200, 50), start, vel_end, 2)
        
        # Draw end effector
        end_pos = positions[-1]
        pygame.draw.circle(self.screen, (255, 100, 100),
                         (int(end_pos[0]), int(end_pos[1])), 6)
        
        # Update and draw particles
        self.particles.emit(end_pos, (255, 200, 100), count=1)
        self.particles.draw(self.screen)
        
        # Update trail
        if self.show_trail:
            self.trail_points.append((int(end_pos[0]), int(end_pos[1])))
            if len(self.trail_points) > self.max_trail_points:
                self.trail_points.pop(0)
        
        # Draw UI elements
        self._draw_ui()
        
        # Draw performance metrics
        fps_text = self.font.render(f"FPS: {int(self.clock.get_fps())}", True, (255, 255, 255))
        self.screen.blit(fps_text, (self.width - 100, self.height - 30))
        
        pygame.display.flip()
        
    def _draw_gradient_line(self, start: Tuple[float, float], end: Tuple[float, float], 
                           color: Tuple[int, int, int]) -> None:
        """Draw a line with enhanced gradient effect"""
        steps = 15
        for i in range(steps):
            t = i / steps
            start_pos = (start[0] + (end[0] - start[0]) * t,
                        start[1] + (end[1] - start[1]) * t)
            end_pos = (start[0] + (end[0] - start[0]) * (t + 1/steps),
                      start[1] + (end[1] - start[1]) * (t + 1/steps))
            
            # Enhanced gradient with glow effect
            alpha = 255 - int(155 * (i / steps))
            base_color = tuple(int(c * alpha/255) for c in color)
            
            # Draw main line
            pygame.draw.line(self.screen, base_color, start_pos, end_pos, 5)
            
            # Draw glow effect
            glow_surface = pygame.Surface((self.width, self.height), pygame.SRCALPHA)
            glow_color = (*color, 50)  # Semi-transparent
            pygame.draw.line(glow_surface, glow_color, start_pos, end_pos, 9)
            self.screen.blit(glow_surface, (0, 0))

# Example usage with enhanced features
if __name__ == "__main__":
    import random
    
    # Example of how to use the enhanced visualizer
    joint_lengths = [120, 100, 80, 60]  # Length of each arm segment
    visualizer = RobotArmVisualizer(joint_lengths)
    
    # Animation loop example with smooth motion and interactive features
    angle = 0
    target_pos = None
    
    while visualizer.update():
        if not visualizer.target_position:  # Only animate if not in IK mode
            # Generate smooth, natural-looking motion
            angles = [
                180 * math.sin(angle) * math.cos(angle * 0.5),
                0 * math.cos(angle) * math.sin(angle * 0.3),
                0 * math.sin(angle * 2) * math.cos(angle * 0.7),
                0 * math.cos(angle * 2) * math.sin(angle * 0.9)
            ]
            
            # Update joint velocities and accelerations
            for i, (new_angle, joint) in enumerate(zip(angles, visualizer.joints)):
                prev_angle = joint.angle
                joint.velocity = (new_angle - prev_angle) * 60  # approx velocity in degrees/sec
                joint.acceleration = (joint.velocity - getattr(joint, 'prev_velocity', 0)) * 60
                joint.prev_velocity = joint.velocity
                
            visualizer.update_angles(angles)
            angle += 0.0002
        
    pygame.quit()