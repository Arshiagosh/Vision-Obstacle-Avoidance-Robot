import math

import pytest

from robot.avoidance import clip_ranges, detour_waypoint, escape_heading, rebound_angle, wrap_deg

DIRECTIONS = [0, 30, 60, 90, 120, 150, 180]


def test_nothing_in_range_means_straight_ahead():
    assert rebound_angle(DIRECTIONS, [None] * 7, 20) == pytest.approx(90)


def test_blocked_on_the_right_turns_left():
    distances = [5, 6, 8, 10, None, None, None]
    alpha = rebound_angle(DIRECTIONS, distances, 20)
    assert alpha > 90
    assert alpha == pytest.approx(10560 / 89)


def test_blocked_on_the_left_turns_right():
    distances = [None, None, None, 10, 8, 6, 5]
    assert rebound_angle(DIRECTIONS, distances, 20) == pytest.approx(180 - 10560 / 89)


def test_far_readings_count_as_free_space():
    # 25 cm is outside a 20 cm bubble, so it must not weigh more than "nothing there"
    assert clip_ranges([25, None, 3, -1], 20) == [20, 20, 3, 0]
    assert rebound_angle(DIRECTIONS, [25, None, None, None, None, None, None], 20) == pytest.approx(90)


def test_boxed_in_keeps_heading():
    assert rebound_angle(DIRECTIONS, [0] * 7, 20) == 90


def test_lengths_must_match():
    with pytest.raises(ValueError):
        rebound_angle(DIRECTIONS, [None] * 6, 20)


@pytest.mark.parametrize(
    "angle, expected",
    [(0, 0), (190, -170), (-190, 170), (180, 180), (-180, 180), (540, 180), (-90, -90)],
)
def test_wrap_deg(angle, expected):
    assert wrap_deg(angle) == pytest.approx(expected)


def test_escape_heading_is_relative_to_current_heading():
    assert escape_heading(0, 90) == pytest.approx(0)
    assert escape_heading(0, 120) == pytest.approx(30)
    assert escape_heading(170, 150) == pytest.approx(-130)


def test_detour_waypoint():
    x, y = detour_waypoint(10, 0, 90, 20)
    assert (x, y) == pytest.approx((10, 20))
    x, y = detour_waypoint(0, 0, 45, 20)
    assert (x, y) == pytest.approx((20 / math.sqrt(2), 20 / math.sqrt(2)))
