import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# IP Camera URL (replace with your URL)
camera_url = "http://192.168.129.35:8080/video"

# Open the camera stream
cap = cv2.VideoCapture(camera_url)

# Minimum area threshold
MIN_AREA = 1050

# Desired width and height for the window
window_width = 800
window_height = 600

# Conversion factor (adjust as needed)
PIXEL_TO_METRIC = 0.001  # Example: 1 pixel = 0.01 meters
SCALING_FACTOR = 0.2  # Scale down the movement (0.01 to 0.90)

# Target Position Publisher
class TargetPositionPublisher(Node):
    def __init__(self):
        super().__init__('target_position_publisher')
        self.publisher_ = self.create_publisher(String, 'target_position', 10)
        self.get_logger().info('Target Position Publisher Node Started')

    def publish_target(self, x, y):
        msg = String()
        msg.data = f"{x} {y}"
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing target position: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    publisher = TargetPositionPublisher()

    try:
        while rclpy.ok():
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

                        # If the aspect ratio is near 1, treat it as a square
                        if 0.95 <= aspectRatio <= 1.05:
                            shape = "Square"
                        else:
                            shape = "Rectangle"

                        # Calculate the center of the shape
                        center_x = x + w // 2
                        center_y = y + h // 2

                        # Convert the pixel coordinates to metric (meters)
                        metric_x = center_x * PIXEL_TO_METRIC
                        metric_y = center_y * PIXEL_TO_METRIC

                        # Apply scaling factor to make the movement smaller
                        scaled_x = metric_x * SCALING_FACTOR
                        scaled_y = metric_y * SCALING_FACTOR

                        # Publish the scaled target position in metric units
                        publisher.publish_target(scaled_x, scaled_y)

                        # Draw the contour and label
                        cv2.drawContours(frame, [approx], 0, (0, 255, 0), 3)
                        cv2.putText(frame, f"{shape} Center: ({center_x}, {center_y})",
                                    (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

                        # Debugging: Print the center coordinates in metric units
                        print(f"Detected {shape} with center in metric coordinates: ({scaled_x}, {scaled_y})")

            # Resize the frame to the smaller window size
            frame_resized = cv2.resize(frame, (window_width, window_height))

            # Display the video feed
            cv2.imshow("Live Shape Detection", frame_resized)

            # Exit when 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass

    finally:
        # Release resources
        cap.release()
        cv2.destroyAllWindows()
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
