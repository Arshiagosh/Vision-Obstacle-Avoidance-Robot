# Constants for Ultrasonic sensor pins
TRIG_FRONT = 27  # Trigger pin for front ultrasonic sensor
ECHO_FRONT = 22  # Echo pin for front ultrasonic sensor
TRIG_RIGHT = 23  # Right ultrasonic
ECHO_RIGHT = 24
TRIG_LEFT = 6    # Left ultrasonic
ECHO_LEFT = 5

# Servo pin for controlling the camera
SERVO_PIN = 18  # GPIO pin for SG90 servo

# Camera and object detection constants
KNOWN_DISTANCE = 30.0  # cm (known distance for camera calibration)
KNOWN_WIDTH = 6.75     # cm (object width)
FOCAL_LENGTH = 915.555 # px, the value the robot ran with during the tests
LOWER_GREEN = [35, 40, 40]  # Lower bound for green color in HSV
UPPER_GREEN = [85, 255, 255]  # Upper bound for green color in HSV

# UART Serial port configuration
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
TIMEOUT = 150

# Sensor angle positions for obstacle avoidance
THETA_SENSORS = [0, 30, 60, 90, 120, 150, 180]

# Maximum range for obstacle detection in cm
MAX_OBSTACLE_DISTANCE = 20.0

# Delay times
SERVO_STABILIZE_DELAY = 1  # Time to wait after moving the servo
