import cv2
from serial_communication import SerialCommunication
from ultrasonic_sensors import UltrasonicSensors
from servo_control import ServoControl
from camera_processing import CameraProcessing
from obstacle_avoidance import ObstacleAvoidance
from constants import *

def main():
    # Initialize components
    serial_comm = SerialCommunication()
    ultrasonic_sensors = UltrasonicSensors()
    ultrasonic_sensors.setup_ultrasonic()
    servo_control = ServoControl()
    camera_processing = CameraProcessing()
    obstacle_avoidance = ObstacleAvoidance()

    # Get target coordinates from the user
    target_x = float(input("Enter target X in cm: "))
    target_y = float(input("Enter target Y in cm: "))
    target_phi = float(input("Enter target Phi in degrees: "))

    # Send initial target coordinates to Arduino
    serial_comm.send_target_coordinates(target_x, target_y, target_phi)

    # Process reference image to find focal length
    ref_image = cv2.imread("rf.png")
    ref_image_obj_width = camera_processing.obj_data(ref_image)
    focal_length_found = camera_processing.focal_length_finder(KNOWN_DISTANCE, KNOWN_WIDTH, ref_image_obj_width)

    # Main angles and distances
    angles = [10, 40, 70, 100, 130]
    distances = []

    # Right ultrasonic measurement
    right_distance = ultrasonic_sensors.measure_distance(TRIG_RIGHT, ECHO_RIGHT)
    print(f"Right Ultrasonic Distance: {right_distance:.2f} cm")
    distances.append(right_distance if right_distance <= MAX_OBSTACLE_DISTANCE else None)

    # Measure distances using camera and ultrasonic sensors
    for angle in angles:
        servo_control.move_servo(angle)
        camera_distance = camera_processing.calculate_distance_from_object()
        if camera_distance is not None:
            print(f"Object detected at {angle} degrees, Distance: {camera_distance:.2f} cm")
        distances.append(camera_distance if camera_distance <= MAX_OBSTACLE_DISTANCE else None)

    # Left ultrasonic measurement
    left_distance = ultrasonic_sensors.measure_distance(TRIG_LEFT, ECHO_LEFT)
    print(f"Left Ultrasonic Distance: {left_distance:.2f} cm")
    distances.append(left_distance if left_distance <= MAX_OBSTACLE_DISTANCE else None)

    # Calculate avoidance angle and rotate
    avoid_phi = obstacle_avoidance.find_avoid(distances, THETA_SENSORS)
    serial_comm.rotate_to_phi(avoid_phi)

    # Continue receiving odometry data
    try:
        while True:
            odometry = serial_comm.receive_odometry_data()
            if odometry:
                x, y, phi = odometry
                print(f"Received odometry data -> X: {x}, Y: {y}, Phi: {phi}")
    except KeyboardInterrupt:
        print("Program interrupted by user")
    finally:
        servo_control.cleanup_servo()
        camera_processing.cleanup_camera()
        serial_comm.close_serial()
        ultrasonic_sensors.cleanup_ultrasonic()

if __name__ == "__main__":
    main()
