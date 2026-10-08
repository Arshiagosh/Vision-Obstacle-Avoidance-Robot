#!/usr/bin/env python3
"""High-level controller of the robot (runs on the Raspberry Pi).

Sends the goal to the Arduino, which does the actual driving, and keeps an
eye on the three ultrasonic sensors. When something gets closer than 20 cm:

  1. stop,
  2. scan: right and left ultrasonics, plus the camera at five angles on the servo,
  3. pick an escape direction with the bubble rebound rule,
  4. turn there and drive a short detour (20 cm),
  5. send the original goal again.

Usage (from the raspberry_pi folder):

    python3 main.py 200 0                # go to (200, 0) cm, avoiding obstacles on the way
    python3 main.py 100 -50 --phi 90     # ...and end up facing 90 deg
    python3 main.py 100 0 --no-avoid     # plain go-to-goal, no sensors needed

Every run is logged to runs/ in the same format as data/logs.
"""

from __future__ import annotations

import argparse
import logging
import time
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

from robot import config
from robot.avoidance import detour_waypoint, escape_heading, rebound_angle
from robot.serial_link import ORIENTATION_REACHED, TARGET_REACHED, SerialLink

log = logging.getLogger("robot")

Reading = Tuple[float, Optional[float]]  # (direction in deg, distance in cm or None)


def obstacle_ahead(sensors) -> bool:
    return any(d is not None and d < config.STOP_DISTANCE_CM for d in sensors.read_all())


def scan(sensors, servo, camera) -> List[Reading]:
    """Free distance in the seven directions, from right (0 deg) to left (180 deg)."""
    readings: List[Reading] = [(config.RIGHT_SENSOR_DEG, sensors.right.read())]
    for direction, servo_angle in config.CAMERA_SCAN:
        servo.move(servo_angle)
        readings.append((direction, camera.measure_distance()))
    readings.append((config.LEFT_SENSOR_DEG, sensors.left.read()))
    servo.move(dict(config.CAMERA_SCAN)[90])  # look ahead again
    return readings


def avoid(link: SerialLink, sensors, servo, camera, turn_timeout: float = 10.0) -> None:
    """Stop, scan, turn to the escape direction and drive the detour."""
    link.stop()
    time.sleep(0.5)  # let it come to a full stop before looking around

    readings = scan(sensors, servo, camera)
    directions = [direction for direction, _ in readings]
    distances = [distance for _, distance in readings]
    alpha = rebound_angle(directions, distances, config.SCAN_RANGE_CM)

    pose = link.pose
    heading = escape_heading(pose.heading_deg if pose else 0.0, alpha)
    log.info(
        "scan %s -> rebound %.1f deg, new heading %.1f deg",
        ", ".join(f"{t:.0f}:{'-' if d is None else f'{d:.0f}'}" for t, d in readings),
        alpha,
        heading,
    )

    link.rotate(heading)
    if not link.wait_for(ORIENTATION_REACHED, turn_timeout):
        log.warning("turn didn't finish in %.0f s, going on anyway", turn_timeout)

    pose = link.pose
    if pose is None:
        return
    wx, wy = detour_waypoint(pose.x, pose.y, heading, config.DETOUR_CM)
    link.goto(wx, wy, heading)
    if not link.wait_for(ORIENTATION_REACHED, turn_timeout):
        log.warning("detour didn't finish in %.0f s, going on anyway", turn_timeout)


def drive_to(
    link: SerialLink,
    goal: Tuple[float, float, float],
    sensors=None,
    servo=None,
    camera=None,
    timeout: float = 120.0,
) -> bool:
    """Go to the goal, avoiding obstacles if the sensors are given. True if it got there."""
    link.goto(*goal)
    at_position = False
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        event = link.poll_event()
        if event == TARGET_REACHED:
            at_position = True  # only the final turn left, no need to watch for obstacles
        elif event == ORIENTATION_REACHED:
            return True

        if sensors is not None and not at_position and obstacle_ahead(sensors):
            log.info("obstacle ahead")
            avoid(link, sensors, servo, camera)
            link.goto(*goal)

        time.sleep(0.05)

    link.stop()
    log.warning("gave up after %.0f s", timeout)
    return False


def main() -> None:
    parser = argparse.ArgumentParser(description="Drive the robot to a goal, avoiding obstacles on the way.")
    parser.add_argument("x", type=float, help="goal x in cm")
    parser.add_argument("y", type=float, help="goal y in cm")
    parser.add_argument("--phi", type=float, default=0.0, help="final heading in degrees (default 0)")
    parser.add_argument("--no-avoid", action="store_true", help="plain go-to-goal, don't use the sensors")
    parser.add_argument("--port", default=config.SERIAL_PORT, help=f"serial port (default {config.SERIAL_PORT})")
    parser.add_argument("--log", help="where to write the odometry log (default: runs/<time>_goal_<x>_<y>.txt)")
    parser.add_argument("--timeout", type=float, default=120.0, help="give up after this many seconds")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")

    log_path = args.log
    if log_path is None:
        Path("runs").mkdir(exist_ok=True)
        log_path = f"runs/{datetime.now():%Y%m%d_%H%M%S}_goal_{args.x:g}_{args.y:g}.txt"

    sensors = servo = camera = None
    pi = None
    if not args.no_avoid:
        import pigpio

        from robot.servo import CameraServo
        from robot.ultrasonic import UltrasonicArray
        from robot.vision import Camera

        pi = pigpio.pi()
        sensors = UltrasonicArray(pi)
        servo = CameraServo(pi)
        camera = Camera()
        servo.move(dict(config.CAMERA_SCAN)[90])

    link = SerialLink(args.port, log_path=log_path)
    link.start()
    try:
        reached = drive_to(link, (args.x, args.y, args.phi), sensors, servo, camera, args.timeout)
        pose = link.pose
        if pose:
            log.info(
                "%s at x=%.1f cm, y=%.1f cm, phi=%.1f deg",
                "done," if reached else "stopped",
                pose.x,
                pose.y,
                pose.heading_deg,
            )
    except KeyboardInterrupt:
        log.info("interrupted, stopping")
        link.stop()
    finally:
        link.close()
        if camera is not None:
            camera.close()
        if servo is not None:
            servo.close()
        if pi is not None:
            pi.stop()
        log.info("log saved to %s", log_path)


if __name__ == "__main__":
    main()
