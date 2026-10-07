/*
 * GotoXYPHI_UART - low-level controller of the robot (Arduino Uno)
 *
 * Takes a goal pose (x, y, phi) from the Raspberry Pi over UART, drives the
 * robot there using wheel-encoder odometry, then turns in place to the final
 * heading. It also streams odometry back to the Pi every 50 ms.
 *
 * Control structure (all loops tick every 10 ms):
 *
 *   goal -> [v PID] -+                          +-> [right wheel PID] -> L298N -> motor
 *           [w PID] -+-> (v, w) -> wheel speeds -+-> [left wheel PID]  -> L298N -> motor
 *    ^                                                                             |
 *    +------------------ odometry (x, y, phi) <------ encoders <-------------------+
 *
 * Serial protocol (115200 baud, one command per line):
 *   "x,y,phi"     go to (x, y) in cm, then rotate to phi in degrees
 *   "ROTATE,phi"  rotate in place to the absolute heading phi (degrees)
 *   "STOP"        stop right where you are
 *
 * Replies:
 *   "x,y,phi,targetL,targetR,speedL,pwmL,speedR,pwmR"  telemetry, every 50 ms
 *   "Target Reached."                                  position reached
 *   "Orientation Reached."                             final heading reached
 *
 * Arshia Goshtasbi - BSc final project, IUST, 1403
 */

#include <util/atomic.h>

// --- Pins -------------------------------------------------------------------
// L298N inputs. Speed is set with PWM directly on these pins, ENA/ENB only
// act as enables.
const uint8_t LEFT_IN1 = 11;
const uint8_t LEFT_IN2 = 10;
const uint8_t RIGHT_IN1 = 6;
const uint8_t RIGHT_IN2 = 5;

// Encoders: channel A goes to an interrupt pin, channel B is read inside the
// ISR to get the direction.
const uint8_t RIGHT_ENC_A = 2;
const uint8_t RIGHT_ENC_B = 8;
const uint8_t LEFT_ENC_A = 3;
const uint8_t LEFT_ENC_B = 7;

// --- Robot geometry ---------------------------------------------------------
const float PULSES_PER_REV = 2625.0;  // encoder pulses per wheel revolution
const float WHEEL_RADIUS = 3.0;       // cm
const float WHEEL_BASE = 20.5;        // distance between the wheels, cm

// --- Control settings -------------------------------------------------------
const unsigned long CONTROL_PERIOD_MS = 10;
const unsigned long TELEMETRY_PERIOD_MS = 50;

const float POSITION_TOLERANCE = 5.0;  // cm
const float ANGLE_TOLERANCE = 0.03;    // rad (~1.7 deg)
const float MAX_LINEAR_CMD = 300.0;    // limit on the v PID output
const float MAX_TURN_CMD = 30.0;       // limit on the w PID output while turning in place

// More pulses than this in one 10 ms tick means faster than the motor's top
// speed (~360 rpm), so it's treated as an encoder glitch and the tick is skipped.
const int MAX_PULSES_PER_TICK = 158;

// --- PID --------------------------------------------------------------------
// Note: the D term is the raw difference between two samples (not divided by
// dt). All the gains below were tuned on the robot this way, so it stays.
class PIDController {
 public:
  PIDController(float kp, float ki, float kd) : kp_(kp), ki_(ki), kd_(kd) {}

  float compute(float target, float current, float dt) {
    float error = target - current;
    integral_ += error * dt;
    float derivative = error - prevError_;
    prevError_ = error;
    return kp_ * error + ki_ * integral_ + kd_ * derivative;
  }

 private:
  float kp_, ki_, kd_;
  float integral_ = 0;
  float prevError_ = 0;
};

// Outer loops of the go-to-goal behaviour
PIDController linearPID(8, 2, 1.5);    // distance to goal -> v
PIDController angularPID(10, 0, 1.5);  // heading error    -> w

// --- Wheels -----------------------------------------------------------------
struct Wheel {
  uint8_t in1, in2;
  PIDController pid;
  volatile long pulses;  // counted in the ISR, cleared every control tick
  float target;          // speed set-point
  float rawSpeed;        // rpm, straight from the encoder (for telemetry)
  float speed;           // rpm, low-pass filtered (used by the PID and odometry)
  float prevRawSpeed;
  int pwm;
};

// Wheel speed loops: motor model from MATLAB's System Identification Toolbox,
// gains from PID Tuner, then touched up on the robot.
Wheel rightWheel = {RIGHT_IN1, RIGHT_IN2, PIDController(4, 0.25, 0.005), 0, 0, 0, 0, 0, 0};
Wheel leftWheel = {LEFT_IN1, LEFT_IN2, PIDController(4, 0.2, 0.005), 0, 0, 0, 0, 0, 0};

void rightEncoderISR() { rightWheel.pulses += digitalRead(RIGHT_ENC_B) ? 1 : -1; }
void leftEncoderISR() { leftWheel.pulses += digitalRead(LEFT_ENC_B) ? -1 : 1; }  // mounted mirrored

// dir: 1 forward, -1 backward, 0 coast
void setMotor(const Wheel &w, int dir, int pwm) {
  if (dir == 1) {
    analogWrite(w.in1, 0);
    analogWrite(w.in2, pwm);
  } else if (dir == -1) {
    analogWrite(w.in1, pwm);
    analogWrite(w.in2, 0);
  } else {
    analogWrite(w.in1, 0);
    analogWrite(w.in2, 0);
  }
}

// One tick of the inner speed loop for a single wheel.
void runWheel(Wheel &w, float dt) {
  long pulses;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pulses = w.pulses;
    w.pulses = 0;
  }

  if (abs(pulses) > MAX_PULSES_PER_TICK) {
    setMotor(w, 0, 0);
    return;
  }

  w.rawSpeed = ((float)pulses / PULSES_PER_REV) * (60.0 / dt);  // rpm
  // first-order low-pass on the speed (the raw estimate is very noisy at 10 ms)
  w.speed = 0.854 * w.speed + 0.0728 * w.rawSpeed + 0.0728 * w.prevRawSpeed;
  w.prevRawSpeed = w.rawSpeed;

  float u = w.pid.compute(w.target, w.speed, dt);
  w.pwm = min((int)fabs(u), 255);
  setMotor(w, u < 0 ? -1 : 1, w.pwm);
}

void resetWheel(Wheel &w) {
  setMotor(w, 0, 0);
  w.target = 0;
  w.rawSpeed = 0;
  w.speed = 0;
  w.prevRawSpeed = 0;
  w.pwm = 0;
}

// --- Odometry ---------------------------------------------------------------
float x = 0, y = 0, phi = 0;  // cm, cm, rad

float normalizeAngle(float a) { return atan2(sin(a), cos(a)); }

// distance covered by a wheel turning at `rpm` for `dt` seconds
float arcLength(float rpm, float dt) { return (2 * PI * WHEEL_RADIUS * rpm / 60.0) * dt; }

void updateOdometry(float dRight, float dLeft) {
  float dCenter = (dRight + dLeft) / 2.0;
  phi = normalizeAngle(phi + (dRight - dLeft) / WHEEL_BASE);
  x += dCenter * cos(phi);
  y += dCenter * sin(phi);
}

// Run both speed loops and integrate the odometry.
void driveWheels(float dt) {
  runWheel(rightWheel, dt);
  runWheel(leftWheel, dt);
  updateOdometry(arcLength(rightWheel.speed, dt), arcLength(leftWheel.speed, dt));
}

// Unicycle (v, w) -> wheel set-points. The outer gains were tuned with this
// exact scaling, so the set-points end up in the same units as the measured
// wheel speed.
void setWheelTargets(float v, float w) {
  rightWheel.target = (2 * v + w * WHEEL_BASE) / (2 * WHEEL_RADIUS);
  leftWheel.target = (2 * v - w * WHEEL_BASE) / (2 * WHEEL_RADIUS);
}

// --- Behaviours -------------------------------------------------------------
enum Mode { IDLE, GOTO, ROTATE };
Mode mode = IDLE;

float goalX = 0, goalY = 0;  // cm
float goalPhi = 0;           // rad
unsigned long lastDriveMs = 0;  // last go-to-goal tick
unsigned long lastTurnMs = 0;   // last rotate-in-place tick

void stopRobot() {
  resetWheel(rightWheel);
  resetWheel(leftWheel);
  mode = IDLE;
}

void startGoto(float gx, float gy, float gphiDeg) {
  goalX = gx;
  goalY = gy;
  goalPhi = gphiDeg * DEG_TO_RAD;
  mode = GOTO;
}

void startRotate(float phiRad) {
  goalPhi = phiRad;
  mode = ROTATE;
}

// Go-to-goal: steer towards (goalX, goalY); once there, turn to goalPhi.
void gotoTarget() {
  float dx = goalX - x;
  float dy = goalY - y;
  float distance = sqrt(dx * dx + dy * dy);

  if (distance < POSITION_TOLERANCE) {
    stopRobot();
    Serial.println(F("Target Reached."));
    delay(100);  // let it settle before turning
    startRotate(goalPhi);
    return;
  }

  unsigned long now = millis();
  if (now - lastDriveMs < CONTROL_PERIOD_MS) return;
  float dt = (now - lastDriveMs) / 1000.0;
  lastDriveMs = now;

  driveWheels(dt);

  dx = goalX - x;
  dy = goalY - y;
  distance = sqrt(dx * dx + dy * dy);
  float headingError = normalizeAngle(atan2(dy, dx) - phi);

  float v = constrain(linearPID.compute(distance, 0, dt), -MAX_LINEAR_CMD, MAX_LINEAR_CMD);
  float w = angularPID.compute(headingError, 0, dt);
  setWheelTargets(v, w);
}

// Rotate in place to goalPhi.
void turnToHeading() {
  float error = normalizeAngle(goalPhi - phi);

  if (fabs(error) < ANGLE_TOLERANCE) {
    stopRobot();
    Serial.println(F("Orientation Reached."));
    return;
  }

  unsigned long now = millis();
  if (now - lastTurnMs < CONTROL_PERIOD_MS) return;
  lastTurnMs = now;

  // Heads-up: dt here is measured from the last *drive* tick, not from the
  // previous turn tick. That came from the original code and is technically a
  // bug, but the wheel loops lean on it: the growing dt makes them push harder
  // and harder, which is what gets the wheels through the motor's dead band
  // when the turn command gets small (with a "correct" dt the turn can stall
  // in the dead band). The price is that the heading odometry gets a bit
  // optimistic during the turn. All the final-heading results were recorded
  // like this, so it stays until someone adds proper dead-band compensation.
  float dt = (now - lastDriveMs) / 1000.0;

  float w = constrain(angularPID.compute(error, 0, dt), -MAX_TURN_CMD, MAX_TURN_CMD);
  setWheelTargets(0, w);
  driveWheels(dt);
}

// --- Serial commands --------------------------------------------------------
char line[48];
uint8_t lineLength = 0;

void handleCommand(char *cmd) {
  if (strncmp(cmd, "ROTATE", 6) == 0) {
    char *comma = strchr(cmd, ',');
    if (comma) {
      // start the dt ramp fresh, like right after a drive (see turnToHeading)
      lastDriveMs = lastTurnMs = millis();
      startRotate(atof(comma + 1) * DEG_TO_RAD);
    }
  } else if (strcmp(cmd, "STOP") == 0) {
    stopRobot();
  } else {
    char *c1 = strchr(cmd, ',');
    char *c2 = c1 ? strchr(c1 + 1, ',') : NULL;
    if (c1 && c2) startGoto(atof(cmd), atof(c1 + 1), atof(c2 + 1));
  }
}

// Non-blocking line reader (readStringUntil() could stall the control loop).
void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLength > 0) {
        line[lineLength] = '\0';
        handleCommand(line);
        lineLength = 0;
      }
    } else if (lineLength < sizeof(line) - 1) {
      line[lineLength++] = c;
    }
  }
}

void sendTelemetry() {
  Serial.print(x);
  Serial.print(',');
  Serial.print(y);
  Serial.print(',');
  Serial.print(phi);
  Serial.print(',');
  Serial.print(leftWheel.target);
  Serial.print(',');
  Serial.print(rightWheel.target);
  Serial.print(',');
  Serial.print(leftWheel.rawSpeed);
  Serial.print(',');
  Serial.print(leftWheel.pwm);
  Serial.print(',');
  Serial.print(rightWheel.rawSpeed);
  Serial.print(',');
  Serial.println(rightWheel.pwm);
}

// --- Main -------------------------------------------------------------------
unsigned long lastTelemetryMs = 0;

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  pinMode(RIGHT_ENC_A, INPUT);
  pinMode(RIGHT_ENC_B, INPUT);
  pinMode(LEFT_ENC_A, INPUT);
  pinMode(LEFT_ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);

  delay(3000);  // give the Pi time to open the port
}

void loop() {
  readSerial();

  if (mode == GOTO)
    gotoTarget();
  else if (mode == ROTATE)
    turnToHeading();

  unsigned long now = millis();
  if (now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = now;
    sendTelemetry();
  }
}
