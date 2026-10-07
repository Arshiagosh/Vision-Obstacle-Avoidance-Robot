#include <util/atomic.h>

int flagStop = 0;

volatile double targetX = 0.0;
volatile double targetY = 0.0;
volatile double targetPhi_g = 0.0;

// Robot state variables
volatile float x = 0.0;
volatile float y = 0.0;
volatile float phi = 0.0;

// Control gains
const double positionTolerance = 5.0; // Tolerance for reaching the goal position (cm)
const double angleTolerance = 0.03;    // Tolerance for reaching the goal orientation (radians)

// Motor pins
const int IN3 = 6;
const int IN4 = 5;
const int IN1 = 11;
const int IN2 = 10;

// Encoder pulse pins
const int pulsePinR1 = 2;
const int pulsePinR2 = 8;
const int pulsePinL1 = 3;
const int pulsePinL2 = 7;

const float pulsePerRev = 2625.0; // Encoder pulses per revolution
const float wheelRadius = 3.0;    // Wheel radius in cm
const float wheelBase = 20.5;     // Distance between wheels (track width) in cm

// Globals
volatile long pos_i_right = 0;
volatile long pos_i_left = 0;
float v1Filt = 0, v1Prev = 0;
float v2Filt = 0, v2Prev = 0;
unsigned long lastControlTime = 0; // Timer for 10ms intervals
unsigned long lastControlTimeTheta = 0;
// Define target velocities for both motors
volatile float targetVelocityRight = 0; // Adjust as needed
volatile float targetVelocityLeft = 0;  // Adjust as needed

volatile float velocityLeft = 0;
volatile float velocityRight = 0;
volatile int pwmLeft = 0;
volatile int pwmRight = 0;

// PID Class
class PIDController {
  public:
    float kp, ki, kd;
    float eintegral, e_old;
    
    PIDController(float kp_, float ki_, float kd_) {
      kp = kp_;
      ki = ki_;
      kd = kd_;
      eintegral = 0;
      e_old = 0;
    }

    float compute(float targetVelocity, float currentVelocity, float deltaT) {
      float e = targetVelocity - currentVelocity;  // Error term
      eintegral += e * deltaT;                     // Integral term
      float e_dot = (e - e_old);                   // Derivative term (change in error)
      e_old = e;                                   // Update old error

      float controlSignal = kp * e + ki * eintegral + kd * e_dot; // PID equation
      return controlSignal;
    }
};

// PID Controllers
PIDController rightMotorPID(4, 0.25, 0.005);
PIDController leftMotorPID(4, 0.2, 0.005);
PIDController vPID(8, 2, 1.5);
PIDController omegaPID(10, 0, 1.5);


// Right encoder ISR
void readEncoderRight() {
  int b = digitalRead(pulsePinR2);
  int increment = (b > 0) ? 1 : -1;
  pos_i_right += increment;
}

// Left encoder ISR
void readEncoderLeft() {
  int b = digitalRead(pulsePinL2);
  int increment = (b > 0) ? -1 : 1;
  pos_i_left += increment;
}

// Function to normalize the angle
float normalizeAngle(float angle) {
  return atan2(sin(angle), cos(angle));
}

// Function to calculate distance traveled
float calculateDistance(float RPM, float time) {
  return (2 * PI * wheelRadius * RPM / 60.0) * time;
}

void setMotor(int dir, int pwmVal, int in1, int in2) {
  if (dir == 1) {
    analogWrite(in1, 0);
    analogWrite(in2, pwmVal);
  } else if (dir == -1) {
    analogWrite(in1, pwmVal);
    analogWrite(in2, 0);
  } else {
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  }
}

// Function to control the motors
void controlMotor(PIDController& pid, volatile long& pos_i, int in1, int in2, float& vFilt, float& vPrev, float targetVelocity, float deltaT) {
  // Calculate the pulse difference since the last check
  int deltaPulse = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    deltaPulse = pos_i;
    pos_i = 0;  // Reset pulse count to avoid overflow
  }

  // Check if the position change is within bounds (absolute value <= 158)
  if (abs(deltaPulse) > 158) {
    setMotor(0, 0, in1, in2);
    return; // Skip the update if the change is too large
  }

  // Convert pulses to velocity (RPM)
  float velocity = ((float)deltaPulse / pulsePerRev) * (60.0 / deltaT);

  // Low-pass filter for smoother velocity reading
  vFilt = 0.854 * vFilt + 0.0728 * velocity + 0.0728 * vPrev;
  vPrev = velocity;

  // PID control
  float controlSignal = pid.compute(targetVelocity, vFilt, deltaT);

  // Determine motor direction and power
  int dir = (controlSignal < 0) ? -1 : 1;
  int pwm = (int)fabs(controlSignal);
  pwm = (pwm > 255) ? 255 : pwm;

  // Set motor
  setMotor(dir, pwm, in1, in2);
  if (in1 == IN1)
  {
    //Left motor
    velocityLeft = velocity;
    pwmLeft =  pwm;
  } 
  else if (in1 == IN3)
  {
    //Right motor
    velocityRight = velocity;
    pwmRight =  pwm;
  }
}

// Function to update robot position
void updateOdometry(float Dr, float Dl) {
  float Dc = (Dr + Dl) / 2.0;
  // Update orientation
  phi = phi + (Dr - Dl) / wheelBase;
  phi = normalizeAngle(phi);
  
  x = x + Dc * cos(phi);
  y = y + Dc * sin(phi);
}
// Function to rotate the robot to a specific angle
void gotoTheta(double targetPhi_g) {
  unsigned long currentTime = millis();
  if (currentTime - lastControlTimeTheta >= 10 && flagStop == 1  && abs(normalizeAngle(targetPhi_g - phi)) > angleTolerance) {
    
    float deltaT = (currentTime - lastControlTime) / 1000.0;
    lastControlTimeTheta = currentTime;

    // Calculate the orientation error
    float orientationError = normalizeAngle(targetPhi_g - phi);

    // PID control for angular velocity
    float targetOmega = omegaPID.compute(orientationError, 0, deltaT);

    if (targetOmega > 0){
        targetOmega = constrain(targetOmega, 0, 30);
      }
      else{
        targetOmega = constrain(targetOmega, -30, 0);
      }

    // Set motor speeds for in-place rotation
    targetVelocityRight = targetOmega * wheelBase / (2 * wheelRadius);
    targetVelocityLeft = -targetOmega * wheelBase / (2 * wheelRadius);

    controlMotor(rightMotorPID, pos_i_right, IN3, IN4, v1Filt, v1Prev, targetVelocityRight, deltaT);
    controlMotor(leftMotorPID, pos_i_left, IN1, IN2, v2Filt, v2Prev, targetVelocityLeft, deltaT);

    float Dr = calculateDistance(v1Filt, deltaT);
    float Dl = calculateDistance(v2Filt, deltaT);
    updateOdometry(Dr, Dl);
  } else if (abs(normalizeAngle(targetPhi_g - phi)) < angleTolerance){
    // Once the orientation is correct, stop the motors
    setMotor(0, 0, IN1, IN2);
    setMotor(0, 0, IN3, IN4);
    flagStop = 2;
    v1Filt = 0; v1Prev = 0;
    v2Filt = 0; v2Prev = 0;
    targetVelocityRight = 0;
    targetVelocityLeft = 0;
    velocityLeft = 0; velocityRight = 0;
    pwmLeft = 0; pwmRight = 0;
    Serial.println("Orientation Reached.");
  }
}

// Function to navigate to a target point (targetX, targetY, targetPhi)
void navigateToTarget(double targetX, double targetY, double targetPhi_g) {
  unsigned long currentTime = millis();
  float distanceToGoal = sqrt(pow(targetX - x, 2) + pow(targetY - y, 2));
  // Similar to your existing control loop logic
  if (currentTime - lastControlTime >= 10 && flagStop == 0 && distanceToGoal > positionTolerance){
    float deltaT = (currentTime - lastControlTime) / 1000.0;
    lastControlTime = currentTime;

    // Motor control logic
    controlMotor(rightMotorPID, pos_i_right, IN3, IN4, v1Filt, v1Prev, targetVelocityRight, deltaT);
    controlMotor(leftMotorPID, pos_i_left, IN1, IN2, v2Filt, v2Prev, targetVelocityLeft, deltaT);

    float Dr = calculateDistance(v1Filt, deltaT);
    float Dl = calculateDistance(v2Filt ,deltaT);
    updateOdometry(Dr, Dl);

    float phi_d = atan2(targetY - y, targetX - x);
    distanceToGoal = sqrt(pow(targetX - x, 2) + pow(targetY - y, 2));
    float orientationError = normalizeAngle(phi_d - phi);

    if (distanceToGoal > positionTolerance) {
      // Velocity and angular velocity control
      float targetV = vPID.compute(distanceToGoal, 0, deltaT);
      float targetOmega = omegaPID.compute(orientationError, 0, deltaT);

      if (targetV > 0){
        targetV = constrain(targetV, 0, 300);
      }
      else{
        targetV = constrain(targetV, -300, 0);
      }

      // Set motor speeds
      targetVelocityRight = (2 * targetV + targetOmega * wheelBase) / (2 * wheelRadius);
      targetVelocityLeft = (2 * targetV - targetOmega * wheelBase) / (2 * wheelRadius);

    }
  }
  else if (flagStop == 1 && distanceToGoal < positionTolerance){
  gotoTheta(targetPhi_g*3.1415/180);
  } 
  else if (flagStop == 0 && distanceToGoal < positionTolerance) {
  setMotor(0, 0, IN1, IN2);
  setMotor(0, 0, IN3, IN4);
  flagStop = 1;
  v1Filt = 0; v1Prev = 0;
  v2Filt = 0; v2Prev = 0;
  targetVelocityRight = 0;
  targetVelocityLeft = 0;
  velocityLeft = 0; velocityRight = 0;
  pwmLeft = 0; pwmRight = 0;
  Serial.println("Target Reached.");
  delay(100);
  gotoTheta(targetPhi_g*3.1415/180.0);
  }
}




// UART reading and parsing function
void readTargetCoordinates() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // Remove any extra whitespace

    if (input.startsWith("ROTATE")) {
      // Handle rotation command
      String rotationAngle = input.substring(input.indexOf(',') + 1);
      targetPhi_g = rotationAngle.toDouble();
      flagStop = 1;  // Stop navigation and start rotation
      //Serial.println("Rotation command received.");
    }
    else if (input.equals("STOP")) {
      // Handle stop command
      flagStop = 2;  // Stop all movement
      setMotor(0, 0, IN1, IN2);
      setMotor(0, 0, IN3, IN4);
      v1Filt = 0; v1Prev = 0;
      v2Filt = 0; v2Prev = 0;
      targetVelocityRight = 0;
      targetVelocityLeft = 0;
      velocityLeft = 0; velocityRight = 0;
      pwmLeft = 0; pwmRight = 0;
      //Serial.println("Stop command received.");
    }
    else {
      // Handle target coordinates
      int commaIndex1 = input.indexOf(',');
      int commaIndex2 = input.lastIndexOf(',');

      if (commaIndex1 != -1 && commaIndex2 != -1) {
        targetX = input.substring(0, commaIndex1).toDouble();
        targetY = input.substring(commaIndex1 + 1, commaIndex2).toDouble();
        targetPhi_g = input.substring(commaIndex2 + 1).toDouble();

        flagStop = 0;  // Reset flag to allow new movement
        //Serial.println("Target coordinates received.");
      } else {
        //Serial.println("Invalid input received.");
      }
    }
  }
}
void setup() {
  Serial.begin(115200);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(pulsePinR1, INPUT); pinMode(pulsePinR2, INPUT);
  pinMode(pulsePinL1, INPUT); pinMode(pulsePinL2, INPUT);
  attachInterrupt(digitalPinToInterrupt(pulsePinR1), readEncoderRight, RISING);
  attachInterrupt(digitalPinToInterrupt(pulsePinL1), readEncoderLeft, RISING);

  delay(3000);
}
unsigned long previousPrintTime = 0;  // Variable to store the last time data was printed
void loop() {
  readTargetCoordinates();  // Continuously check for new coordinates via UART
  /*Serial.print(targetX);
  Serial.print(" ");
  Serial.println(targetY);*/
  navigateToTarget(targetX, targetY, targetPhi_g);
  
  // Get the current time in milliseconds
  unsigned long currentTimePrint = millis();

  // Check if 50ms have passed since the last print
  if (currentTimePrint - previousPrintTime >= 50) {
    // Update the last print time
    previousPrintTime = currentTimePrint;

    // Print x, y, and phi in one line using a single Serial.println statement
    Serial.println(String(x) + "," + String(y) + "," + String(phi) + "," + String(targetVelocityLeft) + ","+ String(targetVelocityRight) + "," +String(velocityLeft) + "," + String(pwmLeft) + "," + String(velocityRight) + "," + String(pwmRight));
  }

}
