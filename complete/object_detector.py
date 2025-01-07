import cv2
import numpy as np

# IP Camera URL (replace with your URL)
camera_url = "http://192.168.129.35:8080/video"

# Open the camera stream
cap = cv2.VideoCapture(camera_url)

# Minimum area threshold
MIN_AREA = 300

while True:
    # Capture a frame
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture video. Exiting...")
        break

    # Convert to grayscale
    imgGrey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply adaptive thresholding
    thresh = cv2.adaptiveThreshold(imgGrey, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

    # Find contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Iterate over contours
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > MIN_AREA:
            # Approximate the contour
            approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
            
            if len(approx) == 4:  # Detect squares/rectangles
                x, y, w, h = cv2.boundingRect(approx)
                aspectRatio = float(w) / h

                # Calculate the center of the shape
                center_x = x + w // 2
                center_y = y + h // 2

                if 0.95 <= aspectRatio <= 1.05:  # Square
                    shape = "Square"
                else:  # Rectangle
                    shape = "Rectangle"

                # Draw the contour and label
                cv2.drawContours(frame, [approx], 0, (0, 255, 0), 3)
                cv2.putText(frame, f"{shape} Center: ({center_x}, {center_y})", 
                            (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

                # Debugging: Print the center coordinates
                print(f"Detected {shape} with center coordinates: ({center_x}, {center_y})")

    # Display the video feed
    cv2.imshow("Live Shape Detection", frame)

    # Exit when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
