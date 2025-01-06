import cv2
import numpy as np
import pygame

# Initialize Pygame
pygame.init()

# Load the image using OpenCV
image = cv2.imread('/home/merlix/Documents/GitHub/RoboT_ARmSimulator/complete/white_square.png')

# Resize the image for better visualization (optional)
image = cv2.resize(image, (800, 600))

# Convert the image to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Apply GaussianBlur to reduce noise
blurred = cv2.GaussianBlur(gray, (5, 5), 0)

# Use Canny edge detection
edges = cv2.Canny(blurred, 50, 150)

# Find contours
contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Detect squares
squares = []
for contour in contours:
    # Approximate the contour to a polygon
    epsilon = 0.04 * cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon, True)

    # Check if the polygon has four vertices and is convex
    if len(approx) == 4 and cv2.isContourConvex(approx):
        # Calculate the bounding rectangle and aspect ratio
        x, y, w, h = cv2.boundingRect(approx)
        aspect_ratio = float(w) / h

        # Consider it a square if the aspect ratio is roughly 1
        if 0.9 <= aspect_ratio <= 1.1:
            squares.append({
                'points': approx.reshape(-1, 2).tolist(),  # Convert to list of tuples
                'center': (x + w // 2, y + h // 2),  # Calculate center of the square
                'area': w * h  # Calculate area for sorting
            })

# Sort squares by area (largest first) and pick the largest one
largest_square = None
if squares:
    squares = sorted(squares, key=lambda s: s['area'], reverse=True)
    largest_square = squares[0]  # Use the largest square

# Prepare the Pygame window
window_width, window_height = image.shape[1], image.shape[0]
screen = pygame.display.set_mode((window_width, window_height))
pygame.display.set_caption("Square Detection")

# Convert the OpenCV image (BGR) to a Pygame-friendly format (RGB)
image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
image_surface = pygame.surfarray.make_surface(np.rot90(image_rgb))

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Clear the screen
    screen.fill((0, 0, 0))

    # Draw the image on the screen (first layer)
    screen.blit(image_surface, (0, 0))

    # Draw the largest detected square (if any)
    if largest_square:
        points = largest_square['points']
        center = largest_square['center']

        # Draw the square outline (green lines)
        pygame.draw.polygon(screen, (0, 255, 0), points, 3)

        # Draw the square center (red dot)
        pygame.draw.circle(screen, (255, 0, 0), center, 5)

    # Update the display (ensure highlights are drawn on top of the image)
    pygame.display.flip()

# Quit Pygame
pygame.quit()
