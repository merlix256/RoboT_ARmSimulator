import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import math

# Robot arm parameters
joint_angles = [0, 0, 0]  # Joint angles in degrees
link_lengths = [2, 2, 2]  # Length of each segment
camera_angle = [0, 0]  # Camera rotation angles (yaw, pitch)

# Initialize PyGame and OpenGL
def initialize():
    """Initialize the PyGame window and OpenGL settings."""
    pygame.init()
    display = (800, 600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)
    glTranslatef(0.0, -3.0, -15)

def draw_axes():
    """Draw the X, Y, and Z axes for reference."""
    glBegin(GL_LINES)

    # X-axis (red)
    glColor3f(1, 0, 0)
    glVertex3f(0, 0, 0)
    glVertex3f(5, 0, 0)

    # Y-axis (green)
    glColor3f(0, 1, 0)
    glVertex3f(0, 0, 0)
    glVertex3f(0, 5, 0)

    # Z-axis (blue)
    glColor3f(0, 0, 1)
    glVertex3f(0, 0, 0)
    glVertex3f(0, 0, 5)

    glEnd()

def draw_grid():
    """Draw a grid on the XZ plane."""
    glColor3f(0.5, 0.5, 0.5)
    glBegin(GL_LINES)
    grid_size = 10
    step = 1
    for i in range(-grid_size, grid_size + 1, step):
        glVertex3f(i, 0, -grid_size)
        glVertex3f(i, 0, grid_size)
        glVertex3f(-grid_size, 0, i)
        glVertex3f(grid_size, 0, i)
    glEnd()

def draw_robot_arm(joint_angles, link_lengths):
    """Draw the robotic arm with 3D solid objects for each segment."""
    x, y, z = 0, 0, 0

    for i, (angle, length) in enumerate(zip(joint_angles, link_lengths)):
        # Convert angle to radians
        angle_rad = math.radians(angle)

        # Calculate new joint position
        x_new = x + length * math.cos(angle_rad)
        y_new = y + length * math.sin(angle_rad)

        # Draw the segment as a 3D cylinder
        draw_cylinder((x, y, z), (x_new, y_new, z), 0.1)

        # Update position
        x, y = x_new, y_new

def draw_cylinder(start, end, radius):
    """Draw a 3D cylinder between two points."""
    x1, y1, z1 = start
    x2, y2, z2 = end

    # Calculate the vector and its length
    vx, vy, vz = x2 - x1, y2 - y1, z2 - z1
    length = math.sqrt(vx**2 + vy**2 + vz**2)

    # Calculate rotation axis and angle
    if length > 0.0001:
        axis = (-vy, vx, 0)
        angle = math.degrees(math.acos(vz / length))
    else:
        axis = (1, 0, 0)
        angle = 0

    # Save current matrix state
    glPushMatrix()

    # Translate to the start point
    glTranslatef(x1, y1, z1)

    # Rotate to align with the vector
    glRotatef(angle, *axis)

    # Draw cylinder
    quad = gluNewQuadric()
    glColor3f(0.5, 0.5, 0.5)  # Segment color
    gluCylinder(quad, radius, radius, length, 16, 16)

    # Restore matrix state
    glPopMatrix()

def draw_base_cube():
    """Draw a cube at the base of the robotic arm."""
    glPushMatrix()
    glColor3f(0.3, 0.3, 0.8)  # Base cube color
    glTranslatef(0, -0.5, 0)  # Position the cube
    glScalef(2, 1, 2)  # Scale the cube
    
    glBegin(GL_QUADS)

    # Front face
    glVertex3f(-0.5, -0.5,  0.5)
    glVertex3f( 0.5, -0.5,  0.5)
    glVertex3f( 0.5,  0.5,  0.5)
    glVertex3f(-0.5,  0.5,  0.5)

    # Back face
    glVertex3f(-0.5, -0.5, -0.5)
    glVertex3f(-0.5,  0.5, -0.5)
    glVertex3f( 0.5,  0.5, -0.5)
    glVertex3f( 0.5, -0.5, -0.5)

    # Top face
    glVertex3f(-0.5,  0.5, -0.5)
    glVertex3f(-0.5,  0.5,  0.5)
    glVertex3f( 0.5,  0.5,  0.5)
    glVertex3f( 0.5,  0.5, -0.5)

    # Bottom face
    glVertex3f(-0.5, -0.5, -0.5)
    glVertex3f( 0.5, -0.5, -0.5)
    glVertex3f( 0.5, -0.5,  0.5)
    glVertex3f(-0.5, -0.5,  0.5)

    # Right face
    glVertex3f( 0.5, -0.5, -0.5)
    glVertex3f( 0.5,  0.5, -0.5)
    glVertex3f( 0.5,  0.5,  0.5)
    glVertex3f( 0.5, -0.5,  0.5)

    # Left face
    glVertex3f(-0.5, -0.5, -0.5)
    glVertex3f(-0.5, -0.5,  0.5)
    glVertex3f(-0.5,  0.5,  0.5)
    glVertex3f(-0.5,  0.5, -0.5)

    glEnd()
    glPopMatrix()

def handle_input(joint_angles, camera_angle):
    """Handle user input to update joint angles and camera rotation."""
    keys = pygame.key.get_pressed()

    # First joint control
    if keys[pygame.K_LEFT]:
        joint_angles[0] -= 1
    if keys[pygame.K_RIGHT]:
        joint_angles[0] += 1

    # Second joint control
    if keys[pygame.K_UP]:
        joint_angles[1] -= 1
    if keys[pygame.K_DOWN]:
        joint_angles[1] += 1

    # Third joint control
    if keys[pygame.K_q]:
        joint_angles[2] -= 1
    if keys[pygame.K_e]:
        joint_angles[2] += 1

    # Camera control
    if keys[pygame.K_a]:
        camera_angle[0] -= 1
    if keys[pygame.K_d]:
        camera_angle[0] += 1
    if keys[pygame.K_w]:
        camera_angle[1] -= 1
    if keys[pygame.K_s]:
        camera_angle[1] += 1

def update_camera(camera_angle):
    """Update the camera view based on rotation angles."""
    glLoadIdentity()
    gluPerspective(45, (800 / 600), 0.1, 50.0)
    glTranslatef(0.0, -3.0, -15)
    glRotatef(camera_angle[1], 1, 0, 0)  # Pitch
    glRotatef(camera_angle[0], 0, 1, 0)  # Yaw

def main():
    """Main function to run the robotic arm simulator."""
    global joint_angles, camera_angle
    initialize()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return

        handle_input(joint_angles, camera_angle)
        update_camera(camera_angle)

        # Clear screen and draw everything
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        draw_axes()
        draw_grid()
        draw_base_cube()
        draw_robot_arm(joint_angles, link_lengths)
        pygame.display.flip()
        pygame.time.wait(10)

if __name__ == "__main__":
    main()
