import serial
import time
import re
from constants import SERIAL_PORT, BAUD_RATE, TIMEOUT

class SerialCommunication:
    def __init__(self):
        # Open the serial port for communication with Arduino
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
        time.sleep(2)  # Wait for the connection to establish

        # Regular expression pattern to match the comma-separated odometry data
        self.pattern = r"(-?\d+\.?\d*),(-?\d+\.?\d*),(-?\d+\.?\d*),(-?\d+\.?\d*),(-?\d+\.?\d*),(-?\d+\.?\d*),(-?\d+\.?\d*),(-?\d+\.?\d*),(-?\d+\.?\d*)"

    def send_target_coordinates(self, x, y, phi):
        """Send the target coordinates and orientation to the Arduino."""
        command = f"{x},{y},{phi}\n"
        self.ser.write(command.encode())
        print(f"Command sent: X = {x}, Y = {y}, Phi = {phi}")

    def rotate_to_phi(self, phi):
        """Send a command to rotate to a specific phi angle."""
        command = f"ROTATE,{phi}\n"
        self.ser.write(command.encode())
        print(f"Rotating to Phi: {phi}")

    def send_stop_command(self):
        """Send a command to stop the robot."""
        command = "STOP\n"
        self.ser.write(command.encode())
        print("Stop command sent.")
    
    def read_odometry(self):
        """Read odometry data from the Arduino."""
        line = self.ser.readline().decode('utf-8', errors='ignore').strip()

        if line == 'Target Reached.':
            print('Target Reached.')
            return None
        if line == 'Orientation Reached.':
            print('Orientation Reached.')
            return None

        # Use the regular expression to extract x, y, and phi
        match = re.match(self.pattern, line)
        if match:
            x = float(match.group(1))
            y = float(match.group(2))
            phi = float(match.group(3))
            targetVelocityLeft = float(match.group(4))
            targetVelocityRight = float(match.group(5))
            velocityLeft = float(match.group(6))
            pwmLeft = float(match.group(7))
            velocityRight = float(match.group(8))
            pwmRight = float(match.group(9))
            return x, y, phi ,  targetVelocityLeft , targetVelocityRight , velocityLeft , pwmLeft , velocityRight , pwmRight
        
        return None

    def close_serial(self):
        """Close the serial connection when done."""
        self.ser.close()
        print("Serial connection closed.")
