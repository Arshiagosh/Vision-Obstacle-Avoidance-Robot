"""Finding the obstacle in a camera frame and estimating how far it is.

The test obstacles were green, so detection is just an HSV color mask:

    BGR frame -> HSV -> green mask -> external contours -> bounding boxes

A blob counts as an obstacle if it's big enough and its bounding box is
mostly green. Its width in pixels then gives the distance with the pinhole
camera model:

    distance = real_width * focal_length / width_in_pixels

where the focal length (in pixels) comes from one reference image taken at a
known distance:

    focal_length = width_in_pixels * known_distance / real_width
"""

from __future__ import annotations

from typing import Optional, Tuple

import cv2
import numpy as np

from . import config


def find_obstacle(frame: np.ndarray, draw: bool = False) -> Optional[Tuple[int, int, int, int]]:
    """Bounding box (x, y, w, h) of the biggest green obstacle in a BGR frame, or None."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(config.HSV_LOWER), np.array(config.HSV_UPPER))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < config.MIN_CONTOUR_AREA:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        green_ratio = cv2.countNonZero(mask[y : y + h, x : x + w]) / float(w * h)
        if green_ratio < config.MIN_GREEN_RATIO:
            continue
        if best is None or area > best[0]:
            best = (area, (x, y, w, h))

    if best is None:
        return None
    box = best[1]
    if draw:
        x, y, w, h = box
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    return box


def distance_from_width(
    width_px: float,
    focal_length_px: float = config.FOCAL_LENGTH_PX,
    real_width_cm: float = config.OBSTACLE_WIDTH_CM,
) -> float:
    return real_width_cm * focal_length_px / width_px


def focal_length_from_reference(
    width_px: float,
    known_distance_cm: float = config.REFERENCE_DISTANCE_CM,
    real_width_cm: float = config.OBSTACLE_WIDTH_CM,
) -> float:
    return width_px * known_distance_cm / real_width_cm


class Camera:
    """Pi camera (OV5647) through picamera2. Frames come out in BGR order, ready for OpenCV."""

    def __init__(self, size: Tuple[int, int] = config.FRAME_SIZE):
        from picamera2 import Picamera2  # only available on the Pi

        self.cam = Picamera2()
        self.cam.configure(self.cam.create_preview_configuration(main={"format": "RGB888", "size": size}))
        self.cam.start()

    def capture(self) -> np.ndarray:
        frame = self.cam.capture_array()
        return cv2.rotate(frame, cv2.ROTATE_180)  # the camera is mounted upside down

    def measure_distance(self) -> Optional[float]:
        """Distance (cm) to the obstacle in front of the camera, or None if there isn't one."""
        box = find_obstacle(self.capture())
        if box is None:
            return None
        return distance_from_width(box[2])

    def close(self) -> None:
        self.cam.stop()
        self.cam.close()
