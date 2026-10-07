import cv2
import numpy as np
from picamera2 import Picamera2
from constants import KNOWN_DISTANCE, KNOWN_WIDTH, LOWER_GREEN, UPPER_GREEN

class CameraProcessing:
    def __init__(self):
        # Initialize the camera
        self.picam2 = Picamera2()
        camera_config = self.picam2.create_preview_configuration(main={"format": "RGB888", "size": (960, 720)})
        self.picam2.configure(camera_config)
        self.picam2.start()

    def is_green(self, roi):
        # Convert ROI to HSV for better green detection
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Define green color range in HSV
        lower_green = np.array([35, 40, 40])  # Adjust this range as per the lighting condition
        upper_green = np.array([85, 255, 255])

        # Create a mask for green pixels
        green_mask = cv2.inRange(hsv_roi, lower_green, upper_green)

        # Calculate the total number of pixels in the ROI
        total_pixels = roi.shape[0] * roi.shape[1]

        # Count the number of green pixels
        green_pixels = cv2.countNonZero(green_mask)

        # If 90% of the pixels are green, return True
        if green_pixels / total_pixels >= 0.8:
            return True
        return False

    def obj_data(self, img):
        """Process the image to detect an object and return its width."""
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(LOWER_GREEN), np.array(UPPER_GREEN))
        _, mask1 = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)
        cnts, _ = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        obj_width = 0
        for c in cnts:
            if cv2.contourArea(c) > 600:
                x, y, w, h = cv2.boundingRect(c)
                roi = img[y:y+h, x:x+w]
                if self.is_green(roi):  # Only consider green objects
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Draw bounding box
                    obj_width = w  # Return the width of the detected object
                    return obj_width
        return obj_width

    def focal_length_finder(self, known_distance, real_width, width_in_rf_image):
        """Calculate and return the focal length based on known measurements."""
        return 915.555

    def capture_frame(self):
        """Capture a frame from the camera and return the rotated image."""
        frame = self.picam2.capture_array()
        frame = cv2.rotate(frame, cv2.ROTATE_180)  # Adjust orientation if needed
        return frame

    def cleanup_camera(self):
        """Release any camera resources and close any OpenCV windows."""
        cv2.destroyAllWindows()
        self.picam2.stop()
        print("Camera cleanup completed.")

    def calculate_distance_from_object(self):
        """Capture a frame, process object width, and calculate the object's distance."""
        frame = self.capture_frame()
        object_width_in_frame = self.obj_data(frame)
        if object_width_in_frame:
            focal_length = self.focal_length_finder(KNOWN_DISTANCE, KNOWN_WIDTH, object_width_in_frame)
            if focal_length:
                distance = (KNOWN_WIDTH * focal_length) / object_width_in_frame
                print(f"Estimated Distance: {distance} cm")
            else:
                print("Focal length not found.")
        else:
            print("Object not detected.")
        return frame
