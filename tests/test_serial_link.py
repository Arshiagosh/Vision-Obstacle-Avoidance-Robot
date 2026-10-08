import queue
import time
from datetime import datetime

import pytest

from robot.serial_link import (
    ORIENTATION_REACHED,
    TARGET_REACHED,
    SerialLink,
    Telemetry,
    format_log_line,
    parse_line,
)


class FakeSerial:
    """Stands in for serial.Serial: lines pushed in come out of readline()."""

    def __init__(self):
        self.incoming = queue.Queue()
        self.written = []
        self.closed = False

    def push(self, line):
        self.incoming.put((line + "\r\n").encode())

    def readline(self):
        try:
            return self.incoming.get(timeout=0.05)
        except queue.Empty:
            return b""

    def write(self, data):
        self.written.append(data.decode())

    def close(self):
        self.closed = True


def test_parse_telemetry():
    t = parse_line("95.06,0.16,-0.03,2.01,-2.01,13.71,14,0.00,6\r\n")
    assert isinstance(t, Telemetry)
    assert (t.x, t.y, t.phi) == (95.06, 0.16, -0.03)
    assert t.pwm_left == 14 and t.pwm_right == 6


def test_parse_events_and_garbage():
    assert parse_line("Target Reached.") == TARGET_REACHED
    assert parse_line("Orientation Reached.\r") == ORIENTATION_REACHED
    assert parse_line("") is None
    assert parse_line("95.06,0.16") is None  # half a line
    assert parse_line("9-5.0,1,2,3,4,5,6,7,8") is None


def test_log_line_matches_the_old_format():
    t = Telemetry(0.48, -0.0, -0.01, 99.75, 100.25, 244.57, 123.0, 267.43, 158.0)
    line = format_log_line(t, datetime(2024, 9, 6, 17, 24, 44, 120000))
    assert line == (
        "2024-09-06 17:24:44.120, X: 0.48, Y: -0.0, Phi: -0.01, "
        "targetVelocityLeft: 99.75, targetVelocityRight: 100.25, velocityLeft: 244.57, "
        "pwmLeft: 123.0, velocityRight: 267.43, pwmRight: 158.0"
    )


def test_commands_go_out_as_lines():
    fake = FakeSerial()
    link = SerialLink(connection=fake)
    link.goto(100, -50, 0)
    link.rotate(-45.5)
    link.stop()
    assert fake.written == ["100.00,-50.00,0.00\n", "ROTATE,-45.50\n", "STOP\n"]


def test_reader_tracks_pose_logs_and_queues_events(tmp_path):
    fake = FakeSerial()
    log_file = tmp_path / "run.txt"
    with SerialLink(connection=fake, log_path=str(log_file)) as link:
        link.goto(10, 0, 0)
        fake.push("1.00,0.00,0.00,10,10,50,80,50,80")
        fake.push("garbage")
        fake.push("9.50,0.10,0.01,0,0,0,0,0,0")
        fake.push("Target Reached.")
        fake.push("Orientation Reached.")
        assert link.wait_for(ORIENTATION_REACHED, timeout=2)
        assert link.pose.x == 9.5
        assert link.pose.heading_deg == pytest.approx(0.573, abs=1e-3)
    assert fake.closed
    lines = log_file.read_text().splitlines()
    assert len(lines) == 2
    assert lines[1].endswith("X: 9.5, Y: 0.1, Phi: 0.01, targetVelocityLeft: 0.0, "
                             "targetVelocityRight: 0.0, velocityLeft: 0.0, pwmLeft: 0.0, "
                             "velocityRight: 0.0, pwmRight: 0.0")


def test_wait_for_times_out():
    with SerialLink(connection=FakeSerial()) as link:
        start = time.monotonic()
        assert not link.wait_for(TARGET_REACHED, timeout=0.2)
        assert time.monotonic() - start < 1


def test_new_command_drops_stale_events():
    fake = FakeSerial()
    with SerialLink(connection=fake) as link:
        fake.push("Orientation Reached.")
        time.sleep(0.2)
        link.rotate(90)  # the old "Orientation Reached." must not count for this turn
        assert not link.wait_for(ORIENTATION_REACHED, timeout=0.2)
