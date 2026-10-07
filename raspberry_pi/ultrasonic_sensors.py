import time
import pigpio
from constants import TRIG_FRONT, ECHO_FRONT, TRIG_RIGHT, ECHO_RIGHT, TRIG_LEFT, ECHO_LEFT

class UltrasonicSensors:
    def __init__(self):
        # Initialize pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise ConnectionError("Unable to connect to pigpio")

    def setup_ultrasonic(self):
        """Set up GPIO for Ultrasonic sensors."""
        self.pi.set_mode(TRIG_FRONT, pigpio.OUTPUT)
        self.pi.set_mode(ECHO_FRONT, pigpio.INPUT)
        self.pi.set_mode(TRIG_RIGHT, pigpio.OUTPUT)
        self.pi.set_mode(ECHO_RIGHT, pigpio.INPUT)
        self.pi.set_mode(TRIG_LEFT, pigpio.OUTPUT)
        self.pi.set_mode(ECHO_LEFT, pigpio.INPUT)

    def measure_distance(self, trig, echo):
        """Measure distance using ultrasonic sensor."""
        self.pi.write(trig, 1)
        time.sleep(0.00001)
        self.pi.write(trig, 0)
        start_time = time.time()
        while self.pi.read(echo) == 0:
            start_time = time.time()
        stop_time = time.time()
        while self.pi.read(echo) == 1:
            stop_time = time.time()
        time_elapsed = stop_time - start_time
        distance = (time_elapsed * 34300) / 2
        return distance

    def cleanup_ultrasonic(self):
        """Cleanup GPIO."""
        self.pi.stop()
