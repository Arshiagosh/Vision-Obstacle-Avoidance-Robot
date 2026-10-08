#!/usr/bin/env python3
"""Grab the reference image for the focal-length calibration.

Put the obstacle exactly REFERENCE_DISTANCE_CM (30 cm) in front of the
camera, run this on the Pi (it needs a screen, VNC is fine) and press:

    c   save the frame to calibration/reference.png
    q   quit without saving

Then run tools/calibrate_focal.py.
"""

import sys
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from robot import config  # noqa: E402
from robot.vision import Camera, find_obstacle  # noqa: E402


def main() -> None:
    camera = Camera()
    out = ROOT / config.REFERENCE_IMAGE
    try:
        while True:
            frame = camera.capture()
            preview = frame.copy()
            find_obstacle(preview, draw=True)
            cv2.imshow("reference (c = save, q = quit)", preview)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("c"):
                out.parent.mkdir(exist_ok=True)
                cv2.imwrite(str(out), frame)
                print(f"saved {out}")
                break
            if key == ord("q"):
                break
    finally:
        camera.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
