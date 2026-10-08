from pathlib import Path

import pytest

from robot.serial_link import Telemetry, format_log_line
from runlog import TELEMETRY_PERIOD_S, load

LOGS = Path(__file__).resolve().parents[1] / "data" / "logs"


@pytest.mark.parametrize("path", sorted(LOGS.rglob("*.txt")), ids=lambda p: p.name)
def test_every_log_in_the_repo_parses(path):
    run = load(path)
    assert len(run) > 50
    assert len(run.x) == len(run.y) == len(run.t) == len(run.pwm_right)


def test_old_logs_use_the_telemetry_period_as_time_axis():
    run = load(LOGS / "goal_100_0.txt")
    assert run.t[1] == pytest.approx(TELEMETRY_PERIOD_S)
    assert (run.x[-1], run.y[-1]) == pytest.approx((95.5, -2.08))  # stopped inside the 5 cm tolerance


def test_obstacle_run_goes_around_and_reaches_the_goal():
    run = load(LOGS / "obstacles_200_0.txt")
    assert run.y.max() > 30  # went around on the left
    assert ((run.x[-1] - 200) ** 2 + run.y[-1] ** 2) ** 0.5 < 5


def test_new_logs_with_milliseconds(tmp_path):
    from datetime import datetime, timedelta

    start = datetime(2026, 1, 1, 12, 0, 0)
    lines = [
        format_log_line(Telemetry(i, 0, 0, 0, 0, 0, 0, 0, 0), start + timedelta(milliseconds=73 * i))
        for i in range(60)
    ]
    path = tmp_path / "run.txt"
    path.write_text("\n".join(lines + ["Target Reached.", "garbage, X: 1"]) + "\n")
    run = load(path)
    assert len(run) == 60
    assert run.t[1] == pytest.approx(0.073)
