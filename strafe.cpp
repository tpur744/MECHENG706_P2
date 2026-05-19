// =====================================================
// STRAFE SYSTEM (EXTRACTED + COMBINED)
// =====================================================

#include <Servo.h>

// ================= MOTOR PINS =================
const byte left_front  = 46;
const byte left_rear   = 47;
const byte right_rear  = 50;
const byte right_front = 51;

// ================= MOTORS =================
Servo leftFrontMotor;
Servo leftRearMotor;
Servo rightRearMotor;
Servo rightFrontMotor;

// ================= SETTINGS =================
int speedVal = 250;
int clampVal = 150;

// ================= HEADING =================
float heading = 0;

// =====================================================
// STRAFE PID CONSTANTS
// =====================================================

// STRAFE RIGHT PID
float Kp_strafeR = 260.0;
float Ki_strafeR = 0.005;
float Kd_strafeR = 0.0;

// STRAFE LEFT PID
float Kp_strafeL = 500.0;
float Ki_strafeL = 100.0;
float Kd_strafeL = 100.0;

// =====================================================
// PID STATE VARIABLES
// =====================================================

// RIGHT STRAFE
float error_sr = 0;
float prevError_sr = 0;
float integral_sr = 0;
unsigned long lastTime_sr = 0;

// LEFT STRAFE
float error_sl = 0;
float prevError_sl = 0;
float integral_sl = 0;
unsigned long lastTime_sl = 0;

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

  leftFrontMotor.attach(left_front);
  leftRearMotor.attach(left_rear);
  rightRearMotor.attach(right_rear);
  rightFrontMotor.attach(right_front);

  stopMotors();

  lastTime_sr = millis();
  lastTime_sl = millis();
}

// =====================================================
// LOOP
// =====================================================

void loop() {

  // Example: strafe right for 2 seconds
  strafeRight(2000);

  delay(1000);

  // Example: strafe left for 2 seconds
  strafeLeft(2000);

  delay(3000);
}

// =====================================================
// STRAFE RIGHT PID
// =====================================================

float computePIDStrafeRight(float target, float current) {

  unsigned long now = millis();

  float dt = (now - lastTime_sr) / 1000.0;

  if (dt <= 0) return 0;

  error_sr = target - current;

  while (error_sr > PI)  error_sr -= 2 * PI;
  while (error_sr < -PI) error_sr += 2 * PI;

  integral_sr += error_sr * dt;

  float derivative = (error_sr - prevError_sr) / dt;

  float output =
      Kp_strafeR * error_sr +
      Ki_strafeR * integral_sr +
      Kd_strafeR * derivative;

  prevError_sr = error_sr;
  lastTime_sr  = now;

  return output;
}

// =====================================================
// STRAFE LEFT PID
// =====================================================

float computePIDStrafeLeft(float target, float current) {

  unsigned long now = millis();

  float dt = (now - lastTime_sl) / 1000.0;

  if (dt <= 0) return 0;

  error_sl = target - current;

  while (error_sl > PI)  error_sl -= 2 * PI;
  while (error_sl < -PI) error_sl += 2 * PI;

  integral_sl += error_sl * dt;

  float derivative = (error_sl - prevError_sl) / dt;

  float output =
      Kp_strafeL * error_sl +
      Ki_strafeL * integral_sl +
      Kd_strafeL * derivative;

  prevError_sl = error_sl;
  lastTime_sl  = now;

  return output;
}

// =====================================================
// STRAFE RIGHT FUNCTION
// =====================================================

void strafeRight(unsigned long durationMS) {

  Serial.println("Strafing Right");

  float targetHeading = heading;

  unsigned long start = millis();

  while (millis() - start < durationMS) {

    updateGyro();

    float rotCorrection =
        computePIDStrafeRight(targetHeading, heading);

    move(0, speedVal, rotCorrection);

    delay(20);
  }

  stopMotors();
}

// =====================================================
// STRAFE LEFT FUNCTION
// =====================================================

void strafeLeft(unsigned long durationMS) {

  Serial.println("Strafing Left");

  float targetHeading = heading;

  unsigned long start = millis();

  while (millis() - start < durationMS) {

    updateGyro();

    float rotCorrection =
        computePIDStrafeLeft(targetHeading, heading);

    move(0, -speedVal, rotCorrection);

    delay(20);
  }

  stopMotors();
}

// =====================================================
// MOTOR MIXING
// =====================================================

int clampMotor(int val) {

  if (val < 1250) val = 1250;
  if (val > 1750) val = 1750;

  if (val > (1500 - clampVal) && val < 1500)
    val = 1500 - clampVal;

  if (val < (1500 + clampVal) && val > 1500)
    val = 1500 + clampVal;

  return val;
}

void move(int forward, int right, int rotate) {

  int lf = clampMotor(1500 + forward + right - rotate);
  int lr = clampMotor(1500 + forward - right - rotate);
  int rr = clampMotor(1500 - forward - right - rotate);
  int rf = clampMotor(1500 - forward + right - rotate);

  leftFrontMotor.writeMicroseconds(lf);
  leftRearMotor.writeMicroseconds(lr);
  rightRearMotor.writeMicroseconds(rr);
  rightFrontMotor.writeMicroseconds(rf);
}

// =====================================================
// STOP MOTORS
// =====================================================

void stopMotors() {

  leftFrontMotor.writeMicroseconds(1500);
  leftRearMotor.writeMicroseconds(1500);
  rightRearMotor.writeMicroseconds(1500);
  rightFrontMotor.writeMicroseconds(1500);
}

// =====================================================
// GYRO UPDATE PLACEHOLDER
// Replace with your IMU code
// =====================================================

void updateGyro() {

  // Example:
  // heading = gyro heading in radians

}
