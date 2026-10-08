"""The high-level loop, driven with fake hardware."""

import math

import pytest

import main
from robot.serial_link import ORIENTATION_REACHED, TARGET_REACHED, Telemetry


def pose(x=0.0, y=0.0, phi_deg=0.0):
    return Telemetry(x, y, math.radians(phi_deg), 0, 0, 0, 0, 0, 0)


class FakeLink:
    """Pretends to be the Arduino: every goal is reached a few polls after it's sent."""

    def __init__(self):
        self.commands = []
        self.pose = pose()
        self._pending = []

    def goto(self, x, y, phi=0.0):
        self.commands.append(("goto", round(x, 2), round(y, 2), round(phi, 2)))
        self._goal = (x, y, phi)
        self._pending = [None, None, None, TARGET_REACHED, ORIENTATION_REACHED]

    def rotate(self, phi):
        self.commands.append(("rotate", round(phi, 2)))
        self._goal = (self.pose.x, self.pose.y, phi)

    def stop(self):
        self.commands.append(("stop",))
        self._pending = []

    def poll_event(self):
        if not self._pending:
            return None
        event = self._pending.pop(0)
        if event == ORIENTATION_REACHED:
            self.pose = pose(*self._goal)
        return event

    def wait_for(self, event, timeout):
        self._pending = []
        self.pose = pose(*self._goal)
        return True


class Sensor:
    def __init__(self, value):
        self.value = value

    def read(self):
        return self.value


class FakeSensors:
    """Something 10 cm ahead on the first look, clear after that."""

    def __init__(self, right=None, left=None):
        self.right, self.left = Sensor(right), Sensor(left)
        self.looks = 0

    def read_all(self):
        self.looks += 1
        if self.looks == 1:
            return 10.0, self.left.value, self.right.value
        return None, None, None  # the detour got it clear of everything


class FakeServo:
    def __init__(self):
        self.angles = []

    def move(self, angle):
        self.angles.append(angle)


class FakeCamera:
    def __init__(self, distances):
        self.distances = list(distances)

    def measure_distance(self):
        return self.distances.pop(0)


@pytest.fixture(autouse=True)
def no_sleeping(monkeypatch):
    monkeypatch.setattr(main.time, "sleep", lambda s: None)


def test_plain_go_to_goal():
    link = FakeLink()
    assert main.drive_to(link, (100, -50, 0))
    assert link.commands == [("goto", 100, -50, 0)]
    assert (link.pose.x, link.pose.y) == (100, -50)


def test_obstacle_on_the_right_detours_left_then_resumes():
    link = FakeLink()
    sensors = FakeSensors(right=5)
    servo = FakeServo()
    camera = FakeCamera([6, 8, 10, None, None])  # 30, 60, 90, 120, 150 deg

    assert main.drive_to(link, (200, 0, 0), sensors, servo, camera)

    alpha = 10560 / 89  # see test_avoidance
    heading = alpha - 90
    wx, wy = 20 * math.cos(math.radians(heading)), 20 * math.sin(math.radians(heading))
    assert link.commands == [
        ("goto", 200, 0, 0),
        ("stop",),
        ("rotate", round(heading, 2)),
        ("goto", round(wx, 2), round(wy, 2), round(heading, 2)),
        ("goto", 200, 0, 0),
    ]
    assert servo.angles == [40, 70, 100, 130, 170, 100]  # scan, then look ahead again


def test_no_obstacle_checks_during_the_final_turn():
    link = FakeLink()
    sensors = FakeSensors()
    sensors.looks = 1  # nothing ahead from the start
    assert main.drive_to(link, (50, 0, 90), sensors, FakeServo(), FakeCamera([]))
    assert link.commands == [("goto", 50, 0, 90)]
