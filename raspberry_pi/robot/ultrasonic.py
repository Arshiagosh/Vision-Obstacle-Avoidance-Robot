"""HC-SR04 ultrasonic sensors, read through the pigpio daemon (sudo pigpiod)."""

from __future__ import annotations

import time
from typing import Optional, Tuple

from . import config


class Ultrasonic:
    """One HC-SR04 on a trigger/echo GPIO pair."""

    def __init__(self, pi, trig: int, echo: int, timeout: float = config.ECHO_TIMEOUT_S):
        import pigpio

        self.pi = pi
        self.trig = trig
        self.echo = echo
        self.timeout = timeout
        pi.set_mode(trig, pigpio.OUTPUT)
        pi.set_mode(echo, pigpio.INPUT)
        pi.write(trig, 0)

    def read(self) -> Optional[float]:
        """Distance in cm, or None if no echo came back in time (nothing in range)."""
        self.pi.gpio_trigger(self.trig, 10, 1)  # 10 us pulse

        deadline = time.perf_counter() + self.timeout
        start = time.perf_counter()
        while self.pi.read(self.echo) == 0:
            start = time.perf_counter()
            if start > deadline:
                return None

        stop = start
        while self.pi.read(self.echo) == 1:
            stop = time.perf_counter()
            if stop - start > self.timeout:
                return None

        return (stop - start) * config.SPEED_OF_SOUND_CM_S / 2


class UltrasonicArray:
    """The three sensors on the robot: front, left and right."""

    def __init__(self, pi=None):
        import pigpio

        self._owns_pi = pi is None
        self.pi = pigpio.pi() if pi is None else pi
        if not self.pi.connected:
            raise ConnectionError("can't reach the pigpio daemon, is pigpiod running?")
        self.front = Ultrasonic(self.pi, config.TRIG_FRONT, config.ECHO_FRONT)
        self.right = Ultrasonic(self.pi, config.TRIG_RIGHT, config.ECHO_RIGHT)
        self.left = Ultrasonic(self.pi, config.TRIG_LEFT, config.ECHO_LEFT)

    def read_all(self) -> Tuple[Optional[float], Optional[float], Optional[float]]:
        """(front, left, right) in cm. Read one after the other so they don't hear each other."""
        return self.front.read(), self.left.read(), self.right.read()

    def close(self) -> None:
        if self._owns_pi:
            self.pi.stop()
