import serial
import pygame
import re

# Initialize serial communication
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
ser.flush()

# Initialize Pygame
pygame.init()

# Screen setup
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Move Box with Trace and Brake")

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 10, 0)
BLUE = (0, 255, 255)
TRACE_COLOR = (100, 255, 255)

# Box properties
box_size = 50
box_x = WIDTH // 2
box_y = HEIGHT // 2
speed = 0.001  # Scaling factor for gyroscope values
trace = []  # List to store the positions of the box for trace

# Define brake threshold for "fully pressed"
FULL_BRAKE_THRESHOLD = 800

# Function to parse the data
def parse_data(input_string):
    gyro_pattern = r"gyro x:\s*(-?\d+)\s*y:\s*(-?\d+)"
    brake_pattern = r"brake:\s*(-?\d+)"
    
    gyro_match = re.search(gyro_pattern, input_string)
    brake_match = re.search(brake_pattern, input_string)

    gyro_values = (0, 0)  # Default values if no match
    brake_value = 0

    if gyro_match:
        gyro_values = tuple(map(int, gyro_match.groups()))
    if brake_match:
        brake_value = int(brake_match.group(1))

    return gyro_values, brake_value

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Read from serial
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').rstrip()
        gyro_values, brake_value = parse_data(line)
        dx = gyro_values[0] * speed  # Change in x
        dy = gyro_values[1] * speed  # Change in y
        
        if brake_value > FULL_BRAKE_THRESHOLD:
            # Only add to trace if brake is not fully pressed
            trace.append((box_x, box_y))

        # Update box position
        box_x += dx
        box_y -= dy

    # Keep the box within screen bounds
    box_x = max(0, min(WIDTH - box_size, box_x))
    box_y = max(0, min(HEIGHT - box_size, box_y))

    # Clear the screen
    screen.fill(BLACK)

    # Draw the trace
    for pos in trace:
        pygame.draw.circle(screen, TRACE_COLOR, (int(pos[0]), int(pos[1])), 2)

    # Draw the box
    pygame.draw.rect(screen, BLUE, (box_x, box_y, box_size, box_size))

    # Update the display
    pygame.display.flip()
    pygame.time.wait(10)

# Quit Pygame
pygame.quit()
