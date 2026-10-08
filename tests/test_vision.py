from pathlib import Path

import cv2
import numpy as np
import pytest

from robot import config
from robot.vision import distance_from_width, find_obstacle, focal_length_from_reference

REFERENCE = Path(__file__).resolve().parents[1] / "raspberry_pi" / config.REFERENCE_IMAGE


def test_finds_the_obstacle_in_the_reference_image():
    frame = cv2.imread(str(REFERENCE))
    assert frame is not None
    x, y, w, h = find_obstacle(frame)
    assert w == pytest.approx(184, abs=3)
    assert h > w  # the box stands upright


def test_no_obstacle_in_a_blank_frame():
    assert find_obstacle(np.zeros((720, 960, 3), np.uint8)) is None


def test_small_green_specks_are_ignored():
    frame = np.zeros((720, 960, 3), np.uint8)
    frame[100:110, 100:110] = (0, 200, 0)  # 10x10 px, below MIN_CONTOUR_AREA
    assert find_obstacle(frame) is None


def test_pinhole_round_trip():
    focal = focal_length_from_reference(206, 30, 6.75)
    assert distance_from_width(206, focal, 6.75) == pytest.approx(30)
    assert distance_from_width(103, focal, 6.75) == pytest.approx(60)  # half the width, twice as far
