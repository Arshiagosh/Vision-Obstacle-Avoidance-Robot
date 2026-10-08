#!/usr/bin/env python3
"""Work out the camera focal length (in pixels) from the reference image.

    python3 tools/calibrate_focal.py                       # uses calibration/reference.png
    python3 tools/calibrate_focal.py my_ref.png --distance 40

Runs anywhere OpenCV is installed, no Pi needed.
"""

import argparse
import sys
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from robot import config  # noqa: E402
from robot.vision import find_obstacle, focal_length_from_reference  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("image", nargs="?", default=str(ROOT / config.REFERENCE_IMAGE))
    parser.add_argument("--distance", type=float, default=config.REFERENCE_DISTANCE_CM,
                        help="obstacle distance in the image, cm (default %(default)s)")
    parser.add_argument("--width", type=float, default=config.OBSTACLE_WIDTH_CM,
                        help="real obstacle width, cm (default %(default)s)")
    parser.add_argument("--save", help="also save the image with the detection drawn on it")
    args = parser.parse_args()

    frame = cv2.imread(args.image)
    if frame is None:
        sys.exit(f"can't read {args.image}")

    box = find_obstacle(frame, draw=True)
    if box is None:
        sys.exit("no green obstacle found in the image")

    focal = focal_length_from_reference(box[2], args.distance, args.width)
    print(f"obstacle box: x={box[0]} y={box[1]} w={box[2]} h={box[3]} px")
    print(f"focal length: {focal:.1f} px   (config.FOCAL_LENGTH_PX is {config.FOCAL_LENGTH_PX})")

    if args.save:
        cv2.imwrite(args.save, frame)
        print(f"saved {args.save}")


if __name__ == "__main__":
    main()
