"""Talking to the Arduino over UART.

Commands, one per line (cm and degrees):
    "x,y,phi"      go to (x, y), then turn to phi
    "ROTATE,phi"   turn in place to the absolute heading phi
    "STOP"         stop right there

The Arduino answers with telemetry every ~50 ms
    "x,y,phi,targetL,targetR,speedL,pwmL,speedR,pwmR"   (phi in radians)
and two status messages: "Target Reached." and "Orientation Reached.".
"""

from __future__ import annotations

import logging
import math
import queue
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Optional, Union

from . import config

log = logging.getLogger(__name__)

TARGET_REACHED = "Target Reached."
ORIENTATION_REACHED = "Orientation Reached."
EVENTS = (TARGET_REACHED, ORIENTATION_REACHED)

# Field names as they appear in the log files (kept from the original logger
# so the old data in data/logs still parses).
LOG_FIELDS = (
    "X", "Y", "Phi",
    "targetVelocityLeft", "targetVelocityRight",
    "velocityLeft", "pwmLeft", "velocityRight", "pwmRight",
)


@dataclass(frozen=True)
class Telemetry:
    x: float  # cm
    y: float  # cm
    phi: float  # rad
    target_left: float
    target_right: float
    speed_left: float  # rpm
    pwm_left: float
    speed_right: float  # rpm
    pwm_right: float

    @property
    def heading_deg(self) -> float:
        return math.degrees(self.phi)

    def values(self) -> tuple:
        return (
            self.x, self.y, self.phi,
            self.target_left, self.target_right,
            self.speed_left, self.pwm_left, self.speed_right, self.pwm_right,
        )


def parse_line(line: str) -> Union[Telemetry, str, None]:
    """A telemetry packet, one of the status messages, or None for anything else."""
    line = line.strip()
    if line in EVENTS:
        return line
    parts = line.split(",")
    if len(parts) != len(LOG_FIELDS):
        return None
    try:
        return Telemetry(*(float(p) for p in parts))
    except ValueError:
        return None


def format_log_line(t: Telemetry, stamp: datetime) -> str:
    """One line of the run log, e.g. '2024-09-06 17:24:44.120, X: 0.48, Y: -0.0, ...'."""
    fields = ", ".join(f"{name}: {value}" for name, value in zip(LOG_FIELDS, t.values()))
    return f"{stamp:%Y-%m-%d %H:%M:%S.%f}"[:-3] + ", " + fields


class SerialLink:
    """Owns the serial port: sends commands, keeps the latest pose, writes the run log.

    A background thread reads everything the Arduino sends. Telemetry updates
    `pose` (and goes to the log file, if there is one); the "...Reached."
    messages are queued so the main loop can wait for them.

    `connection` can be any object with readline()/write()/close(), which is
    how the tests drive it without hardware.
    """

    def __init__(
        self,
        port: str = config.SERIAL_PORT,
        baud: int = config.BAUD_RATE,
        log_path: Optional[str] = None,
        connection=None,
    ):
        if connection is None:
            import serial  # pyserial, only needed on the robot

            connection = serial.Serial(port, baud, timeout=config.SERIAL_TIMEOUT_S)
            time.sleep(2)  # give the link a moment before the first command
        self._conn = connection
        self._log = open(log_path, "a", buffering=1) if log_path else None
        self._lock = threading.Lock()
        self._pose: Optional[Telemetry] = None
        self._events: "queue.Queue[str]" = queue.Queue()
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def __enter__(self) -> "SerialLink":
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, name="serial-reader", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2)
        self._conn.close()
        if self._log is not None:
            self._log.close()

    # --- commands -----------------------------------------------------------

    def goto(self, x: float, y: float, phi_deg: float = 0.0) -> None:
        self._send(f"{x:.2f},{y:.2f},{phi_deg:.2f}")

    def rotate(self, phi_deg: float) -> None:
        self._send(f"ROTATE,{phi_deg:.2f}")

    def stop(self) -> None:
        self._send("STOP")

    def _send(self, command: str) -> None:
        self.clear_events()  # anything older belongs to the previous command
        self._conn.write((command + "\n").encode())
        log.info(">> %s", command)

    # --- state --------------------------------------------------------------

    @property
    def pose(self) -> Optional[Telemetry]:
        with self._lock:
            return self._pose

    def poll_event(self) -> Optional[str]:
        try:
            return self._events.get_nowait()
        except queue.Empty:
            return None

    def wait_for(self, event: str, timeout: float) -> bool:
        """Block until `event` arrives. False on timeout."""
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            try:
                if self._events.get(timeout=remaining) == event:
                    return True
            except queue.Empty:
                return False

    def clear_events(self) -> None:
        while self.poll_event() is not None:
            pass

    # --- reader thread ------------------------------------------------------

    def _read_loop(self) -> None:
        while self._running:
            try:
                raw = self._conn.readline()
            except Exception:  # port went away; nothing sensible left to do here
                log.exception("serial read failed")
                break
            if not raw:
                continue
            message = parse_line(raw.decode("utf-8", errors="ignore"))
            if isinstance(message, Telemetry):
                with self._lock:
                    self._pose = message
                if self._log is not None:
                    self._log.write(format_log_line(message, datetime.now()) + "\n")
            elif message is not None:
                log.info("<< %s", message)
                self._events.put(message)
