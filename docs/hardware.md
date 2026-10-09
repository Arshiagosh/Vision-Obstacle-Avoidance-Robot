# Hardware

<p align="center">
  <img src="images/robot_front.jpg" height="260" alt="Robot, front view">
  &nbsp;
  <img src="images/robot_top.jpg" height="260" alt="Robot, top view">
</p>

Two stacked 16 × 14 cm plates held together by four standoffs. The lower
deck has the Arduino and a breadboard for the wiring; the upper deck has the
Raspberry Pi, the motor driver, the battery, the three ultrasonic sensors and
the camera on its servo. The motors hang under the lower plate on brackets.

It's a differential drive: two driven wheels (3 cm radius, 20.5 cm apart) and
a caster for balance, so it's non-holonomic and can turn in place.

## Parts

| Part | What it does | Notes |
|---|---|---|
| Arduino Uno (ATmega328P) | low-level control: wheel speeds, odometry, go-to-goal | runs [`firmware/`](../firmware/GotoXYPHI_UART/GotoXYPHI_UART.ino) |
| Raspberry Pi 4 Model B (8 GB) | high-level control: sensors, camera, obstacle avoidance | runs [`raspberry_pi/`](../raspberry_pi) |
| 2 × 12 V DC gear motors with encoders | drive | 1:21 gearbox, ~370 rpm at the output, 3 kg·cm, ~1.5 V dead band |
| MITSUMI M25N-2R-14 encoders | wheel odometry | 2625 pulses per wheel revolution (rising edges of channel A) |
| L298N dual H-bridge | motor driver | PWM on the IN pins, its 5 V regulator also powers the Arduino |
| 12 V Li-ion pack (3 cells, 2.5 Ah) | motor power | |
| 3 × HC-SR04 | front / left / right distance | wired to the Pi |
| Raspberry Pi Camera v1.3 (OV5647) | obstacle detection and ranging | 5 MP, 3.6 mm lens, used at 960 × 720 |
| SG90 servo | pans the camera for the scan | 0–180° |
| USB to TTL serial adapter | Pi ↔ Arduino UART | keeps the Pi's 3.3 V pins away from the Uno's 5 V |

## Wiring

### Arduino Uno

| Pin | Connected to |
|---|---|
| D11, D10 | L298N IN1, IN2 (left motor) |
| D6, D5 | L298N IN3, IN4 (right motor) |
| D2 (INT0) | right encoder channel A |
| D8 | right encoder channel B |
| D3 (INT1) | left encoder channel A |
| D7 | left encoder channel B |
| RX, TX | USB-TTL adapter (to the Pi) |

The left encoder is mounted mirrored, so its ISR counts the other way round.

### Raspberry Pi (BCM numbering)

| GPIO | Connected to |
|---|---|
| 27 / 22 | front HC-SR04 trigger / echo |
| 23 / 24 | right HC-SR04 trigger / echo |
| 6 / 5 | left HC-SR04 trigger / echo |
| 18 | SG90 signal |
| CSI port | camera |
| USB | USB-TTL adapter (`/dev/ttyUSB0`) |

All of this lives in [`raspberry_pi/robot/config.py`](../raspberry_pi/robot/config.py)
and at the top of the sketch, so if you rewire something that's the only
place to change.

> **If you rebuild it:** the HC-SR04 echo pin outputs 5 V and the Pi's GPIOs
> are 3.3 V, so put a small voltage divider (e.g. 1 kΩ / 2 kΩ) on each echo line.

## Datasheets

Not included here since they're not mine to redistribute, but these are the
parts to search for: *25GA DC gear motor*, *MITSUMI M25N-2-R-14*, *L298N*,
*HC-SR04*, *Raspberry Pi Camera v1.3*, *SG90*.
