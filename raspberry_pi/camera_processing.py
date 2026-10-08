import cv2
import numpy as np
from picamera2 import Picamera2
from constants import KNOWN_DISTANCE, KNOWN_WIDTH, FOCAL_LENGTH, LOWER_GREEN, UPPER_GREEN

class CameraProcessing:
    def __init__(self):
        # Initialize the camera
        self.picam2 = Picamera2()
        camera_config = self.picam2.create_preview_configuration(main={"format": "RGB888", "size": (960, 720)})
        self.picam2.configure(camera_config)
        self.picam2.start()

    def is_green(self, roi):
        """True if at least 80% of the ROI is green (filters out noisy blobs)."""
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv_roi, np.array(LOWER_GREEN), np.array(UPPER_GREEN))
        total_pixels = roi.shape[0] * roi.shape[1]
        green_pixels = cv2.countNonZero(green_mask)
        return green_pixels / total_pixels >= 0.8

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
        """Focal length in pixels from a reference image of the object at a known distance."""
        return (width_in_rf_image * known_distance) / real_width

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
        """Capture a frame and return the distance (cm) to the green object, or None."""
        frame = self.capture_frame()
        object_width_in_frame = self.obj_data(frame)
        if not object_width_in_frame:
            print("Object not detected.")
            return None
        distance = (KNOWN_WIDTH * FOCAL_LENGTH) / object_width_in_frame
        print(f"Estimated Distance: {distance:.1f} cm")
        return distance
