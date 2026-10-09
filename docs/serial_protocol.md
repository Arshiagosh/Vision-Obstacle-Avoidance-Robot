# Serial protocol (Pi ↔ Arduino)

Plain text over UART, 115200 baud, 8N1, one message per line (`\n`).
Distances are in cm, angles in **degrees** from the Pi, **radians** in the
telemetry (that's what the odometry keeps internally).

## Pi → Arduino

| Command | Example | Meaning |
|---|---|---|
| `x,y,phi` | `200.00,0.00,0.00` | go to (x, y), then turn in place to phi |
| `ROTATE,phi` | `ROTATE,35.50` | turn in place to the absolute heading phi |
| `STOP` | `STOP` | stop immediately, forget the current goal |

Anything else is ignored. A new command replaces whatever the robot was
doing.

## Arduino → Pi

Every 50 ms, whether it's moving or not:

```
x,y,phi,targetL,targetR,speedL,pwmL,speedR,pwmR
95.06,0.16,-0.03,2.01,-2.01,13.71,14,0.00,6
```

| Field | Unit | |
|---|---|---|
| `x`, `y` | cm | odometry position (starts at 0, 0 on power-up) |
| `phi` | rad | heading, wrapped to (−π, π] |
| `targetL`, `targetR` | controller units | wheel speed set-points |
| `speedL`, `speedR` | rpm | raw wheel speed from the encoders |
| `pwmL`, `pwmR` | 0–255 | duty cycle sent to the L298N |

And two status lines:

| Line | When |
|---|---|
| `Target Reached.` | within 5 cm of (x, y); the final turn starts right after |
| `Orientation Reached.` | within 0.03 rad of the final heading; the robot is idle again |

So a normal `x,y,phi` command ends with `Target Reached.` followed by
`Orientation Reached.`, and a `ROTATE` only produces the second one.

## Trying it by hand

With just the Arduino plugged into a computer, any serial terminal works:

```bash
python -m serial.tools.miniterm /dev/ttyUSB0 115200
```

Type `50,0,90` and the robot drives half a meter forward and then turns left.
The Pi side of all this is [`raspberry_pi/robot/serial_link.py`](../raspberry_pi/robot/serial_link.py).
