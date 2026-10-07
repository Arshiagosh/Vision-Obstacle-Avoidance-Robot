# Test-run data

Odometry logs recorded by the Raspberry Pi during the experiments on
16 Shahrivar 1403 (6 September 2024). Every line is one telemetry packet from
the Arduino (sent every ~50 ms), stamped by the Pi when it arrived.

All positions come from wheel-encoder odometry only — there was no external
ground truth (no motion capture, no IMU), so these show what the robot
*thought* it was doing.

## Clean runs

| File | Start → goal `(x, y, φ)` | What happened | Video |
|---|---|---|---|
| `logs/goal_100_0.txt` | (0, 0, 0°) → (100, 0, 0°) | straight run, stops inside the 5 cm tolerance | `media/videos/goal_100_0.mp4` |
| `logs/goal_100_-50.txt` | (0, 0, 0°) → (100, −50, 0°) | turns towards the goal, drives, then rotates in place back to 0° | `media/videos/goal_100_-50.mp4` |
| `logs/goal_100_-50_take1.txt` | (0, 0, 0°) → (100, −50, 0°) | an earlier successful take of the same goal | – |
| `logs/obstacles_200_0.txt` | (0, 0) → (200, 0) | three obstacles on the straight line; the robot stops, scans, turns left around them and reaches the goal | `media/videos/obstacles.mp4` |

The (0, 100) run was filmed too, but its log file came out as all zeros, so
only the video and the figure in the report survive for that one.

## `logs/tuning/`

Raw logs from the tuning sessions on the same day. Several attempts are
appended into the same file (the logger opened files in append mode), so the
position jumps back to zero wherever a new attempt starts. They're kept for
completeness; you probably want the clean runs above.

## Format

```
2024-09-06 17:24:44, X: 0.48, Y: -0.0, Phi: -0.01, targetVelocityLeft: 99.75, targetVelocityRight: 100.25, velocityLeft: 244.57, pwmLeft: 123.0, velocityRight: 267.43, pwmRight: 158.0
```

| Field | Unit | Meaning |
|---|---|---|
| timestamp | – | time the Pi received the line (1 s resolution in these old logs; the new logger writes milliseconds) |
| `X`, `Y` | cm | odometry position, starting at (0, 0) |
| `Phi` | rad | heading, wrapped to (−π, π] |
| `targetVelocityLeft/Right` | controller units | wheel speed set-points from the go-to-goal layer |
| `velocityLeft/Right` | rpm | raw (unfiltered) wheel speed from the encoders |
| `pwmLeft/Right` | 0–255 | PWM duty sent to the L298N |

To plot any of them:

```bash
python analysis/plot_run.py data/logs/obstacles_200_0.txt
```
