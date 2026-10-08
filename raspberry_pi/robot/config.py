"""Every number that's specific to this robot, in one place.

Directions in the avoidance code follow the convention from the thesis: they
are measured from the robot's right side, so 0 deg is right, 90 deg is
straight ahead and 180 deg is left.
"""

# --- Serial link to the Arduino (through a USB-TTL adapter) ----------------
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200
SERIAL_TIMEOUT_S = 1.0

# --- HC-SR04 ultrasonic sensors (BCM pin numbers) --------------------------
TRIG_FRONT, ECHO_FRONT = 27, 22
TRIG_RIGHT, ECHO_RIGHT = 23, 24
TRIG_LEFT, ECHO_LEFT = 6, 5
SPEED_OF_SOUND_CM_S = 34300
ECHO_TIMEOUT_S = 0.03  # no echo after this -> nothing in range

# --- SG90 servo that pans the camera ---------------------------------------
SERVO_PIN = 18
SERVO_SETTLE_S = 1.0  # wait after each move before grabbing a frame

# --- Camera and obstacle detection -----------------------------------------
FRAME_SIZE = (960, 720)
# The test obstacles were green boxes, 6.75 cm wide. Detection is a plain HSV
# color mask, so it only "sees" green things.
OBSTACLE_WIDTH_CM = 6.75
HSV_LOWER = (35, 40, 40)
HSV_UPPER = (85, 255, 255)
MIN_CONTOUR_AREA = 600  # px^2, anything smaller is noise
MIN_GREEN_RATIO = 0.8   # share of the bounding box that has to be green

# Pinhole model: distance = OBSTACLE_WIDTH_CM * FOCAL_LENGTH_PX / width_in_px.
# 915.555 is the value the robot ran with in all the tests. To recalibrate,
# put the obstacle REFERENCE_DISTANCE_CM in front of the camera, then run
# tools/capture_reference.py and tools/calibrate_focal.py.
FOCAL_LENGTH_PX = 915.555
REFERENCE_DISTANCE_CM = 30.0
REFERENCE_IMAGE = "calibration/reference.png"

# --- Obstacle avoidance -----------------------------------------------------
STOP_DISTANCE_CM = 20.0  # any of the three ultrasonics closer than this -> stop and scan
SCAN_RANGE_CM = 20.0     # radius of the "sensitivity bubble", farther counts as free
DETOUR_CM = 20.0         # drive this far in the escape direction, then head for the goal again

RIGHT_SENSOR_DEG = 0
LEFT_SENSOR_DEG = 180
# (direction, servo angle) for the five camera looks. The servo angles are
# the ones used on the robot during the tests.
CAMERA_SCAN = ((30, 40), (60, 70), (90, 100), (120, 130), (150, 170))
