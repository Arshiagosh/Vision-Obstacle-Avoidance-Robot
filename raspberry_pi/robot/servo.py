"""The SG90 servo that pans the camera, driven through pigpio."""

from __future__ import annotations

import time

from . import config


class CameraServo:
    def __init__(self, pi=None, pin: int = config.SERVO_PIN, settle_s: float = config.SERVO_SETTLE_S):
        import pigpio

        self._owns_pi = pi is None
        self.pi = pigpio.pi() if pi is None else pi
        if not self.pi.connected:
            raise ConnectionError("can't reach the pigpio daemon, is pigpiod running?")
        self.pin = pin
        self.settle_s = settle_s

    def move(self, angle: float) -> None:
        """Go to `angle` (0-180 deg) and wait for the servo (and the image) to settle."""
        angle = max(0.0, min(180.0, angle))
        self.pi.set_servo_pulsewidth(self.pin, 500 + angle / 180.0 * 2000)  # 0.5-2.5 ms
        time.sleep(self.settle_s)

    def close(self) -> None:
        self.pi.set_servo_pulsewidth(self.pin, 0)  # stop sending pulses
        if self._owns_pi:
            self.pi.stop()
