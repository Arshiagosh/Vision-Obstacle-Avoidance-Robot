"""Bubble rebound obstacle avoidance.

From I. Susnea, V. Minzu and G. Vasiliu, "Simple, real-time obstacle
avoidance algorithm for mobile robots" (WSEAS CIMMACS, 2009).

Once the robot has stopped in front of something, it measures the free
distance in a few directions. The escape direction is the average of those
directions, each one weighted by how much free space there is that way:

    alpha = sum(theta_i * d_i) / sum(d_i)

Only what's inside the "sensitivity bubble" matters, so every reading is
clipped to the bubble radius and a direction with nothing in range counts as
fully free. Directions use the thesis convention (0 = right, 90 = ahead,
180 = left).
"""

from __future__ import annotations

import math
from typing import Iterable, List, Optional, Sequence, Tuple


def clip_ranges(distances: Iterable[Optional[float]], max_range: float) -> List[float]:
    """Clip readings to the bubble radius. None (nothing seen) counts as max_range."""
    return [max_range if d is None else max(0.0, min(d, max_range)) for d in distances]


def rebound_angle(
    directions_deg: Sequence[float],
    distances: Sequence[Optional[float]],
    max_range: float,
) -> float:
    """Escape direction in degrees (0 = right, 90 = ahead, 180 = left)."""
    if len(directions_deg) != len(distances):
        raise ValueError("need one distance per direction")
    clipped = clip_ranges(distances, max_range)
    total = sum(clipped)
    if total == 0:
        return 90.0  # boxed in on every side, nothing better than straight ahead
    return sum(theta * d for theta, d in zip(directions_deg, clipped)) / total


def wrap_deg(angle: float) -> float:
    """Wrap an angle to (-180, 180]."""
    wrapped = math.fmod(angle + 180.0, 360.0)
    if wrapped <= 0:
        wrapped += 360.0
    return wrapped - 180.0


def escape_heading(current_heading_deg: float, rebound_deg: float) -> float:
    """Absolute heading (odometry frame, degrees) that points along the rebound direction.

    A rebound angle of 90 means "keep going straight", anything above turns
    left (counter-clockwise), anything below turns right.
    """
    return wrap_deg(current_heading_deg + rebound_deg - 90.0)


def detour_waypoint(x: float, y: float, heading_deg: float, distance: float) -> Tuple[float, float]:
    """The point `distance` cm ahead of (x, y) along heading_deg."""
    h = math.radians(heading_deg)
    return x + distance * math.cos(h), y + distance * math.sin(h)
