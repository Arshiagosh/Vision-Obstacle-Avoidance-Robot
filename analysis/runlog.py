"""Reading the odometry logs the Pi writes (data/logs/*.txt and runs/*.txt).

Each line looks like

    2024-09-06 17:24:44.120, X: 0.48, Y: -0.0, Phi: -0.01, targetVelocityLeft: 99.75, ...

The old logs from the thesis only have whole seconds in the timestamp, so
for those the time axis is rebuilt from the telemetry period (50 ms).
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Union

import numpy as np

TELEMETRY_PERIOD_S = 0.05

_LINE = re.compile(r"^(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d(?:\.\d+)?),\s*(.+)$")
_FIELDS = {
    "X": "x",
    "Y": "y",
    "Phi": "phi",
    "targetVelocityLeft": "target_left",
    "targetVelocityRight": "target_right",
    "velocityLeft": "speed_left",
    "pwmLeft": "pwm_left",
    "velocityRight": "speed_right",
    "pwmRight": "pwm_right",
}


@dataclass
class Run:
    name: str
    t: np.ndarray  # s, from the first sample
    x: np.ndarray  # cm
    y: np.ndarray  # cm
    phi: np.ndarray  # rad
    target_left: np.ndarray
    target_right: np.ndarray
    speed_left: np.ndarray  # rpm
    pwm_left: np.ndarray
    speed_right: np.ndarray  # rpm
    pwm_right: np.ndarray

    def __len__(self) -> int:
        return len(self.t)

    @property
    def heading_deg(self) -> np.ndarray:
        return np.degrees(self.phi)

    @property
    def duration(self) -> float:
        return float(self.t[-1]) if len(self.t) else 0.0

    @property
    def path_length(self) -> float:
        return float(np.sum(np.hypot(np.diff(self.x), np.diff(self.y))))


def load(path: Union[str, Path]) -> Run:
    path = Path(path)
    stamps, columns = [], {name: [] for name in _FIELDS.values()}

    for raw in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        match = _LINE.match(raw.strip())
        if not match:
            continue
        values = {}
        for item in match.group(2).split(","):
            key, _, value = item.partition(":")
            key = key.strip()
            if key in _FIELDS:
                try:
                    values[_FIELDS[key]] = float(value)
                except ValueError:
                    break
        if len(values) != len(_FIELDS):
            continue  # truncated or garbled line
        stamps.append(match.group(1))
        for name, value in values.items():
            columns[name].append(value)

    if not stamps:
        raise ValueError(f"no telemetry lines in {path}")

    if all("." in s for s in stamps):
        times = [datetime.strptime(s, "%Y-%m-%d %H:%M:%S.%f") for s in stamps]
        t = np.array([(ts - times[0]).total_seconds() for ts in times])
    else:
        t = np.arange(len(stamps)) * TELEMETRY_PERIOD_S

    return Run(name=path.stem, t=t, **{k: np.array(v) for k, v in columns.items()})
