import pigpio
import time
from constants import SERVO_PIN, SERVO_STABILIZE_DELAY

class ServoControl:
    def __init__(self, pin=SERVO_PIN):
        """Initialize the ServoControl class and connect to the pigpio daemon."""
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("Failed to connect to pigpio daemon.")
        self.servo_pin = pin

    def move_servo(self, angle):
        """Move the servo motor to the specified angle."""
        pulsewidth = 500 + (angle / 180.0) * 2000
        self.pi.set_servo_pulsewidth(self.servo_pin, pulsewidth)
        print(f"Moved to {angle} degrees")
        time.sleep(SERVO_STABILIZE_DELAY)  # Allow time to stabilize after movement

    def cleanup_servo(self):
        """Cleanup the servo control and stop the pigpio connection."""
        self.pi.set_servo_pulsewidth(self.servo_pin, 0)
        self.pi.stop()
        print("Servo control cleaned up.")
