import cv2
import threading
import time
from serial_communication import SerialCommunication
from ultrasonic_sensors import UltrasonicSensors
from servo_control import ServoControl
from camera_processing import CameraProcessing
from obstacle_avoidance import ObstacleAvoidance
from constants import *
x = 0
y = 0
phi = 0
def odometry_thread(serial_comm):
    """Thread function to continuously read odometry data."""
    global x,y,phi
    while True:
        odometry = serial_comm.read_odometry()
        if odometry:
            x, y, phi = odometry
            print(f"Received odometry data -> X: {x}, Y: {y}, Phi: {phi}")

def obstacle_avoidance_mode(sensor, camera_processing, serial_comm, servo, obstacle):
    """Handle obstacle avoidance mode."""
    global phi
    distances = []
    time.sleep(2)
    distance = sensor.measure_distance(TRIG_RIGHT, ECHO_RIGHT)
    distances.append(distance if distance is not None and distance <= (MAX_OBSTACLE_DISTANCE+10) else None)
    # print(distance)
    angles = [40, 70, 100, 130, 170]
    for angle in angles:
        print(angle)
        servo.move_servo(angle)
        #frame = camera_processing.capture_frame()
        # cv2.imshow("FRAME", frame)
        #time.sleep(20)
        #obj_width_in_frame = camera_processing.obj_data(frame)
        # print(obj_width_in_frame)
        time.sleep(1)
        '''if obj_width_in_frame >= 150:
            focal_length_found = 915.555
            camera_distance = (KNOWN_WIDTH * focal_length_found) / obj_width_in_frame
            print(camera_distance)
            distances.append(camera_distance if camera_distance <= (MAX_OBSTACLE_DISTANCE+10) else None)
        else:
            distances.append(None)'''
    
    distance = sensor.measure_distance(TRIG_LEFT, ECHO_LEFT)
    distances.append(distance if distance is not None and distance <= (MAX_OBSTACLE_DISTANCE+10) else None)
    print(distances)
    
    # Calculate avoidance angle
    avoid_phi = obstacle.find_avoid(distances, THETA_SENSORS)
    print('Avoid phi :' + str((avoid_phi+phi*180/3.1415 -90)*10))
    serial_comm.rotate_to_phi(10*(avoid_phi+phi*180/3.14-90))
    time.sleep(2)
    # Wait for the robot to complete avoidance and then resume
    # time.sleep(20)  # Adjust based on how long avoidance should take

def main():
    # Initialize components
    serial_comm = SerialCommunication()
    sensor = UltrasonicSensors()
    camera_processing = CameraProcessing()
    servo_control = ServoControl()
    obstacle_avoidance = ObstacleAvoidance()

    # Get target coordinates from the user
    target_x = float(input("Enter target X in cm: "))
    target_y = float(input("Enter target Y in cm: "))
    target_phi = float(input("Enter target Phi in degrees: "))

    # Send initial target coordinates to Arduino
    serial_comm.send_target_coordinates(target_x, target_y, target_phi)

    # Start odometry reading thread
    odometry_thread_instance = threading.Thread(target=odometry_thread, args=(serial_comm,))
    odometry_thread_instance.daemon = True
    odometry_thread_instance.start()

    
    try:
        while True:
            
            # Continuously check ultrasonic sensors and handle obstacles
            distances = [
                sensor.measure_distance(TRIG_FRONT, ECHO_FRONT),
                sensor.measure_distance(TRIG_RIGHT, ECHO_RIGHT),
                sensor.measure_distance(TRIG_LEFT, ECHO_LEFT)
            ]
            # print(distances)
            if any(d is not None and d < MAX_OBSTACLE_DISTANCE for d in distances):
                print("Obstacle detected! Entering avoidance mode.")
                serial_comm.send_stop_command()  # Send stop command
                obstacle_avoidance_mode(sensor, camera_processing, serial_comm, servo_control, obstacle_avoidance)
                # Send new target coordinates after avoidance
                serial_comm.send_target_coordinates(target_x, target_y, target_phi)
                time.sleep(1)
            else:
                # Continue normal operation (if applicable)
                pass

            # time.sleep(1)  # Adjust as needed

    except KeyboardInterrupt:
        print("Program interrupted by user")
    finally:
        servo_control.cleanup_servo()
        camera_processing.cleanup_camera()
        serial_comm.close_serial()

if __name__ == "__main__":
    main()
