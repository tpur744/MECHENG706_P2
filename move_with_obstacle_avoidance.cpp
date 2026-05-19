// =====================================================
// HEAD TO LIGHT — with IR-guided strafe obstacle avoidance
// =====================================================

#include <Servo.h>
#include <Adafruit_BNO08x.h>

// ================= ULTRASONIC =================
const int trigPin = 48;
const int echoPin = 49;

// ===== FAN =======
const int fanPin = 22;

// ===== Active Drive Tracking =====
int   bestDriveSum     = 0;
float bestDriveHeading = 0;
int   prevSum          = 0;
bool  peakDetected     = false;
bool  servoLocked      = false;

// ================= SERVO =================
Servo scanServo;
const int servoPin    = 9;
const int centreAngle = 120;
const int offsetAngle = 30;
const int servoStep   = 2;
const int servoDelay  = 20;

int  servoAngle     = centreAngle + offsetAngle;
int  servoDirection = -1;
unsigned long lastServoTime = 0;

// ========== LEDS ==========
#define LED_ORANGE  10
#define LED_GREEN   11
#define LED_BLUE    12
#define LED_RED     13

// ================= MOTORS =================
const byte left_front  = 46;
const byte left_rear   = 47;
const byte right_rear  = 50;
const byte right_front = 51;

Servo leftFrontMotor;
Servo leftRearMotor;
Servo rightRearMotor;
Servo rightFrontMotor;

int clampVal = 150;
int speedVal = 250;

// ================= GYRO =================
Adafruit_BNO08x   bno08x(-1);
sh2_SensorValue_t sensorValue;

float gyroZ = 0;
float heading = 0;
unsigned long lastGyroTime = 0;

// ================= PHOTOTRANSISTORS =================
const int sensor1 = A8;
const int sensor2 = A9;
const int sensor3 = A10;
const int sensor4 = A11;

// ================= SCAN RESULTS =================
float bestHeading = 0;
int   bestSum     = 0;

// ================= FORWARD PID =================
float Kp_fwd = 75.0;
float Ki_fwd = 0.02;
float Kd_fwd = 0;

float error_fwd = 0, prevError_fwd = 0, integral_fwd = 0;
unsigned long lastTime_fwd = 0;

// ================= ALIGN PID =================
float Kp_align = 200.0;
float Ki_align = 5;
float Kd_align = 0.0;

float alignPrevError = 0;
float alignIntegral  = 0;
unsigned long alignLastTime = 0;

// =====================================================
// IR SENSOR PINS
// =====================================================
const int LEFT_IR       = A12;
const int RIGHT_IR      = A13;
const int BACK_LEFT_IR  = A14;
const int BACK_RIGHT_IR = A15;

const float IR_alpha      = 0.02;
const int   IR_SAMPLE_AVG = 20;

float leftAvg      = 0;
float rightAvg     = 0;
float backleftAvg  = 0;
float backrightAvg = 0;

// ================= STRAFE PID — RIGHT =================
float Kp_strafeR = 260.0;
float Ki_strafeR = 0.005;
float Kd_strafeR = 0.0;

float error_sr = 0, prevError_sr = 0, integral_sr = 0;
unsigned long lastTime_sr = 0;

// ================= STRAFE PID — LEFT =================
float Kp_strafeL = 500.0;
float Ki_strafeL = 100.0;
float Kd_strafeL = 100.0;

float error_sl = 0, prevError_sl = 0, integral_sl = 0;
unsigned long lastTime_sl = 0;

// =====================================================
// STATE MACHINE
// =====================================================
// *** RESCAN added: a short sweep to re-find the light
//     after strafing, without doing a full 360 ***
enum State { SCANNING, ROTATING, DRIVING, STRAFING, RESCAN, REVERSING, DONE };
State state = SCANNING;

int lightsFound = 0;
const int totalLights = 2;
unsigned long reverseStartTime = 0;

// ================= STRAFE STATE =================
int           strafeDir           = 1;   // +1 = right, -1 = left
float         strafeTargetHeading = 0;
unsigned long strafeStartTime     = 0;

bool          postStrafeForward   = false;
unsigned long postStrafeStartTime = 0;

const unsigned long STRAFE_DURATION_MS     = 2000;
const unsigned long POST_STRAFE_FORWARD_MS = 500;

// *** RESCAN state variables ***
// sweepScanHeading: heading to sweep back toward (the pre-strafe light direction)
// sweepRange: how many radians either side of sweepScanHeading to search
// sweepDir: which way we are currently rotating during the sweep
float         sweepScanHeading = 0;
const float   sweepRange       = 60.0 * PI / 180.0;  // ±60 degrees
int           sweepDir         = 0;
float         sweepStart       = 0;
int           sweepBestSum     = 0;
float         sweepBestHeading = 0;
bool          sweepComplete    = false;

// =====================================================
// SETUP
// =====================================================
void setup() {

  pinMode(fanPin, OUTPUT);
  digitalWrite(fanPin, LOW);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.begin(115200);
  LED_Setup();
  scanServo.attach(servoPin);
  scanServo.write(centreAngle);

  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);

  pinMode(LEFT_IR,       INPUT);
  pinMode(RIGHT_IR,      INPUT);
  pinMode(BACK_LEFT_IR,  INPUT);
  pinMode(BACK_RIGHT_IR, INPUT);

  leftFrontMotor.attach(left_front);
  leftRearMotor.attach(left_rear);
  rightRearMotor.attach(right_rear);
  rightFrontMotor.attach(right_front);
  delay(100);

  stopMotors();

  if (!bno08x.begin_I2C()) {
    Serial.println("IMU FAIL");
    while (1);
  }
  LED_Setup();

  useGyroIntegration();
  delay(200);

  warmupIR();

  lastTime_fwd  = millis();
  alignLastTime = millis();
  lastTime_sr   = millis();
  lastTime_sl   = millis();
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  updateGyro();
  updateIR();

  switch (state) {

    case SCANNING:
      scanForLight();
      stopMotors();
      delay(300);
      error_fwd = 0; prevError_fwd = 0; integral_fwd = 0;
      lastTime_fwd = millis();
      alignIntegral = 0; alignPrevError = 0;
      alignLastTime = millis();
      state = ROTATING;
      break;

    case ROTATING:
      if (rotateToHeading(bestHeading)) {
        delay(200);
        bestDriveHeading = bestHeading;
        bestDriveSum     = 0;
        state = DRIVING;
      }
      break;

    case DRIVING:
      driveForward();
      break;

    case STRAFING:
      doStrafe();
      break;

    // *** New RESCAN case: short sweep to re-acquire light heading ***
    case RESCAN:
      doRescan();
      break;

    case REVERSING: {
      if (millis() - reverseStartTime < 1000) {
        move(-150, 0, 0);
        plotSensors();
      } else {
        stopMotors();
        delay(300);
        bestSum = 0;
        bestDriveSum = 0;
        error_fwd = 0; prevError_fwd = 0; integral_fwd = 0;
        alignIntegral = 0; alignPrevError = 0;
        servoLocked    = false;
        scanServo.write(centreAngle);
        servoAngle     = centreAngle + offsetAngle;
        servoDirection = -1;
        state = SCANNING;
      }
      break;
    }

    case DONE:
      stopMotors();
      plotSensors();
      delay(20);
      break;
  }
}

// =====================================================
// STRAFE STATE HANDLER
// Phase 1: strafe sideways for STRAFE_DURATION_MS
// Phase 2: drive forward for POST_STRAFE_FORWARD_MS
// *** Then: go to RESCAN instead of ROTATING ***
// =====================================================
void doStrafe() {

  updateGyro();
  unsigned long now = millis();

  // ---- Phase 2: post-strafe forward ----
  if (postStrafeForward) {
    if (now - postStrafeStartTime >= POST_STRAFE_FORWARD_MS) {
      stopMotors();
      delay(200);

      // *** Transition to RESCAN, not ROTATING ***
      // Seed the sweep target as the last known light heading
      sweepScanHeading = bestDriveHeading;
      sweepBestSum     = 0;
      sweepBestHeading = bestDriveHeading;
      sweepComplete    = false;

      // *** Determine which way to start sweeping:
      //     rotate toward bestDriveHeading from current heading ***
      float diff = bestDriveHeading - heading;
      while (diff >  PI) diff -= 2 * PI;
      while (diff < -PI) diff += 2 * PI;
      sweepDir   = (diff >= 0) ? 1 : -1;
      sweepStart = heading;

      // Reset PIDs for clean driving after rescan
      error_fwd = 0; prevError_fwd = 0; integral_fwd = 0;
      lastTime_fwd  = millis();
      alignIntegral = 0; alignPrevError = 0;
      alignLastTime = millis();
      integral_sr   = 0; integral_sl = 0;
      postStrafeForward = false;

      state = RESCAN;
      return;
    }
    move(speedVal, 0, 0);
    return;
  }

  // ---- Phase 1: timed strafe ----
  if (now - strafeStartTime >= STRAFE_DURATION_MS) {
    stopMotors();
    delay(100);
    postStrafeForward   = true;
    postStrafeStartTime = millis();
    return;
  }

  float rotCorrection;
  if (strafeDir > 0) {
    rotCorrection = computePIDStrafeRight(strafeTargetHeading, heading);
    move(0, speedVal, 0*(int)rotCorrection);
  } else {
    rotCorrection = computePIDStrafeLeft(strafeTargetHeading, heading);
    move(0, -speedVal, 0*(int)rotCorrection);
  }
}

// =====================================================
// *** RESCAN — sweep ±60° around last known light
//     heading to find the brightest direction.
//     Once the sweep is complete, rotate to the best
//     heading found, then resume DRIVING. ***
//
// Two sub-phases:
//   sweepComplete == false : still rotating through arc
//   sweepComplete == true  : rotating to bestHeading found
// =====================================================
void doRescan() {
  updateGyro();

  // ---- Sub-phase 2: rotate to the best heading found ----
  if (sweepComplete) {
    if (rotateToHeading(sweepBestHeading)) {
      // Locked on — seed drive state and go
      bestHeading      = sweepBestHeading;
      bestDriveHeading = sweepBestHeading;
      bestDriveSum     = 0;
      servoLocked      = false;
      scanServo.write(centreAngle);
      servoAngle     = centreAngle + offsetAngle;
      servoDirection = -1;
      state = DRIVING;
    }
    return;
  }

  // ---- Sub-phase 1: sweep arc, record best sensor sum ----
  // How far have we rotated from the sweep start?
  float rotated = heading - sweepStart;
  while (rotated >  PI) rotated -= 2 * PI;
  while (rotated < -PI) rotated += 2 * PI;

  // Read light sensors
  int v1  = analogRead(sensor1);
  int v2  = analogRead(sensor2);
  int v3  = analogRead(sensor3);
  int v4  = analogRead(sensor4);
  int sum = v1 + v2 + v3 + v4;

  if (sum > sweepBestSum) {
    sweepBestSum     = sum;
    sweepBestHeading = heading;
  }

  LED_Number(2000, sum);
  plotSensors();

  // Check if we've swept the full ±60° arc (total 120°)
  if (abs(rotated) >= 2 * sweepRange) {
    // Sweep done — stop and rotate to best heading found
    stopMotors();
    delay(100);
    sweepComplete = true;
    return;
  }

  // Keep rotating through the arc
  int rotateSpeed = 150;
  move(0, 0, -sweepDir * rotateSpeed);
  delay(20);
}

// =====================================================
// TRIGGER STRAFE
// Reads left/right IR, picks the roomier side.
// *** Saves bestDriveHeading as reference for rescan ***
// =====================================================
void triggerStrafe() {
  float leftDist  = getLeftDistance();
  float rightDist = getRightDistance();

  Serial.print("Obstacle detected! L=");
  Serial.print(leftDist);
  Serial.print("cm  R=");
  Serial.print(rightDist);
  Serial.println("cm");

  // Strafe toward whichever side has more space
  strafeDir = (leftDist > rightDist) ? -1 : 1;

  strafeTargetHeading = heading;
  strafeStartTime     = millis();
  postStrafeForward   = false;

  integral_sr = 0; prevError_sr = 0; lastTime_sr = millis();
  integral_sl = 0; prevError_sl = 0; lastTime_sl = millis();

  stopMotors();
  delay(100);
  state = STRAFING;
}

// =====================================================
// PHASE 1 — 360 scan
// =====================================================
void scanForLight() {
  int rotateSpeed = 220;
  bestSum     = 0;
  bestHeading = heading;

  float lastHeading  = heading;
  float totalRotated = 0.0;
  unsigned long scanStart = millis();

  while (totalRotated < 2 * PI) {
    updateGyro();
    if (millis() - scanStart > 6000) {
      Serial.println("SCAN TIMEOUT");
      break;
    }
    if (totalRotated > (330.0 * PI / 180.0)) rotateSpeed = 80;

    move(0, 0, -rotateSpeed);

    int v1  = analogRead(sensor1);
    int v2  = analogRead(sensor2);
    int v3  = analogRead(sensor3);
    int v4  = analogRead(sensor4);
    int sum = v1 + v2 + v3 + v4;
    LED_Number(2000, sum);

    if (sum > bestSum) {
      bestSum     = sum;
      bestHeading = heading;
    }

    int sumNorm     = map(sum, 0, 4092, 0, 1023);
    int headingPlot = map((int)(heading * 1000),
                          (int)(-PI * 1000), (int)(PI * 1000),
                          0, 1023);
    Serial.print("S1:"); Serial.print(v1);
    Serial.print(" S2:"); Serial.print(v2);
    Serial.print(" S3:"); Serial.print(v3);
    Serial.print(" S4:"); Serial.print(v4);
    Serial.print(" SumNorm:"); Serial.print(sumNorm);
    Serial.print(" Heading:"); Serial.println(headingPlot);

    float delta = heading - lastHeading;
    while (delta >  PI) delta -= 2 * PI;
    while (delta < -PI) delta += 2 * PI;
    if (abs(delta) > 0.001) totalRotated += abs(delta);

    lastHeading = heading;
    delay(20);
  }

  stopMotors();
  while (bestHeading >  PI) bestHeading -= 2 * PI;
  while (bestHeading < -PI) bestHeading += 2 * PI;
}

// =====================================================
// PHASE 2 — Rotate to heading
// =====================================================
bool rotateToHeading(float target) {
  updateGyro();

  float headingError = target - heading;
  while (headingError >  PI) headingError -= 2 * PI;
  while (headingError < -PI) headingError += 2 * PI;

  float absErr = abs(headingError);
  int dir = (headingError > 0) ? 1 : -1;

  plotSensors();
  LED_Number(0.1, absErr);

  if (absErr < (3.0 * PI / 180.0)) {
    stopMotors();
    return true;
  }

  int rotateSpeed = (absErr > (30.0 * PI / 180.0)) ? 250 : 100;
  move(0, 0, -dir * rotateSpeed);
  delay(20);
  return false;
}

// =====================================================
// PHASE 3 — Drive forward toward light
// =====================================================
void driveForward() {
  updateGyro();

  float dist = getDistance();
  int angleFromCentre = servoAngle - centreAngle;

  //LED_Number(100, dist);

  if (!servoLocked && dist < 30.0 && abs(angleFromCentre) < 3) {
    servoLocked = true;
    scanServo.write(centreAngle);
  }

  if (!servoLocked) {
    unsigned long now = millis();
    if (now - lastServoTime >= servoDelay) {
      lastServoTime = now;
      servoAngle += servoDirection * servoStep;
      if (servoAngle <= centreAngle - offsetAngle) {
        servoAngle     = centreAngle - offsetAngle;
        servoDirection = +1;
      } else if (servoAngle >= centreAngle + offsetAngle) {
        servoAngle     = centreAngle + offsetAngle;
        servoDirection = -1;
      }
      scanServo.write(servoAngle);
    }
  }

  // ---- Final stop: reached the light ----
  if (dist < 10.0) {
    stopMotors();
    unsigned long t = millis();
    while (millis() - t < 3000) digitalWrite(fanPin, HIGH);
    digitalWrite(fanPin, LOW);

    lightsFound++;
    if (lightsFound >= totalLights) {
      state = DONE;
      return;
    }
    delay(5000);
    reverseStartTime = millis();
    state = REVERSING;
    return;
  }

  // ---- Obstacle at 15cm — strafe around it ----
  if (analogRead(sensor1) > 800){
    LED("toggle", "orange");
  }
  if (analogRead(sensor2) > 800){
    LED("toggle", "green");
  }
  if (analogRead(sensor3) > 800){
    LED("toggle", "blue");
  }
  if (analogRead(sensor4) > 800){
    LED("toggle", "red");
  }

  if (dist < 15.0 && !(max(max(analogRead(sensor1),analogRead(sensor2)), max(analogRead(sensor3),analogRead(sensor4)))>700)) {
    triggerStrafe();
    return;
  }

  // ---- Normal forward drive ----
  int sum = analogRead(sensor1) + analogRead(sensor2)
          + analogRead(sensor3) + analogRead(sensor4);

  if (sum > bestDriveSum) {
    bestDriveSum = sum;
    float offsetDeg = servoAngle - centreAngle;
    float offsetRad = offsetDeg * (PI / 180.0);
    bestDriveHeading = heading + offsetRad;
    while (bestDriveHeading >  PI) bestDriveHeading -= 2 * PI;
    while (bestDriveHeading < -PI) bestDriveHeading += 2 * PI;
  }

  bestDriveSum = bestDriveSum * 0.995;

  float correction = computePIDForward(bestDriveHeading, heading);
  move(200, 0, (int)correction);
  //LED_Number(0.1, correction);
  plotSensors();
}

// =====================================================
// PLOT HELPER
// =====================================================
void plotSensors() {
  int v1  = analogRead(sensor1);
  int v2  = analogRead(sensor2);
  int v3  = analogRead(sensor3);
  int v4  = analogRead(sensor4);
  int sum = v1 + v2 + v3 + v4;
  int sumNorm     = map(sum, 0, 4092, 0, 1023);
  int headingPlot = map((int)(heading * 1000),
                        (int)(-PI * 1000), (int)(PI * 1000),
                        0, 1023);
  Serial.print("S1:"); Serial.print(v1);
  Serial.print(" S2:"); Serial.print(v2);
  Serial.print(" S3:"); Serial.print(v3);
  Serial.print(" S4:"); Serial.print(v4);
  Serial.print(" SumNorm:"); Serial.print(sumNorm);
  Serial.print(" Heading:"); Serial.println(headingPlot);
}

// =====================================================
// FORWARD PID
// =====================================================
float computePIDForward(float target, float current) {
  unsigned long now = millis();
  float dt = (now - lastTime_fwd) / 1000.0;
  if (dt <= 0) return 0;

  error_fwd = target - current;
  while (error_fwd >  PI) error_fwd -= 2 * PI;
  while (error_fwd < -PI) error_fwd += 2 * PI;

  integral_fwd += error_fwd * dt;
  float derivative = (error_fwd - prevError_fwd) / dt;
  float output = Kp_fwd * error_fwd + Ki_fwd * integral_fwd + Kd_fwd * derivative;

  prevError_fwd = error_fwd;
  lastTime_fwd  = now;
  return output;
}

// =====================================================
// ALIGN PID
// =====================================================
float computeAlignPID(float error) {
  unsigned long now = millis();
  float dt = (now - alignLastTime) / 1000.0;
  if (dt <= 0) return 0;

  alignIntegral += error * dt;
  float derivative = (error - alignPrevError) / dt;
  float output = Kp_align * error + Ki_align * alignIntegral + Kd_align * derivative;

  alignPrevError = error;
  alignLastTime  = now;
  return output;
}

// =====================================================
// STRAFE RIGHT PID
// =====================================================
float computePIDStrafeRight(float target, float current) {
  unsigned long now = millis();
  float dt = (now - lastTime_sr) / 1000.0;
  if (dt <= 0) return 0;

  error_sr = target - current;
  while (error_sr >  PI) error_sr -= 2 * PI;
  while (error_sr < -PI) error_sr += 2 * PI;

  integral_sr += error_sr * dt;
  float derivative = (error_sr - prevError_sr) / dt;
  float output = Kp_strafeR * error_sr + Ki_strafeR * integral_sr + Kd_strafeR * derivative;

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
  while (error_sl >  PI) error_sl -= 2 * PI;
  while (error_sl < -PI) error_sl += 2 * PI;

  integral_sl += error_sl * dt;
  float derivative = (error_sl - prevError_sl) / dt;
  float output = Kp_strafeL * error_sl + Ki_strafeL * integral_sl + Kd_strafeL * derivative;

  prevError_sl = error_sl;
  lastTime_sl  = now;
  return output;
}

// =====================================================
// GYRO
// =====================================================
void updateGyro() {
  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_GYROSCOPE_UNCALIBRATED) {
      gyroZ = sensorValue.un.gyroscope.z;
      unsigned long now = millis();
      float dt = (now - lastGyroTime) / 1000.0;
      heading -= gyroZ * dt;
      while (heading >  PI) heading -= 2 * PI;
      while (heading < -PI) heading += 2 * PI;
      lastGyroTime = now;
    }
  }
}

void useGyroIntegration() {
  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 0);
  delay(10);
  bno08x.enableReport(SH2_GYROSCOPE_UNCALIBRATED);
  delay(50);
  lastGyroTime = millis();
  heading = 0;
}

// =====================================================
// IR SENSORS
// =====================================================
void updateIR() {
  leftAvg =
    IR_alpha * (analogRead(LEFT_IR) * (5.0 / 1024.0)) +
    (1 - IR_alpha) * leftAvg;

  rightAvg =
    IR_alpha * (analogRead(RIGHT_IR) * (5.0 / 1024.0)) +
    (1 - IR_alpha) * rightAvg;

  backleftAvg =
    IR_alpha * (analogRead(BACK_LEFT_IR) * (5.0 / 1024.0)) +
    (1 - IR_alpha) * backleftAvg;

  backrightAvg =
    IR_alpha * (analogRead(BACK_RIGHT_IR) * (5.0 / 1024.0)) +
    (1 - IR_alpha) * backrightAvg;
}

void warmupIR() {
  for (int i = 0; i < 50; i++) {
    updateIR();
    delay(4);
  }
}

float getLeftDistance() {
  return 13.61 / (leftAvg - 0.469);
}

float getRightDistance() {
  return 17.85 / (rightAvg - 0.33);
}

// =====================================================
// MOTOR CONTROL
// =====================================================
int clampMotor(int val) {
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

void stopMotors() {
  leftFrontMotor.writeMicroseconds(1500);
  leftRearMotor.writeMicroseconds(1500);
  rightRearMotor.writeMicroseconds(1500);
  rightFrontMotor.writeMicroseconds(1500);
}

// =====================================================
// ULTRASONIC
// =====================================================
float getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  float duration = pulseIn(echoPin, HIGH);
  return (duration * 0.0343) / 2.0;
}

// =====================================================
// LEDs
// =====================================================
static int _ledPin(const char* color) {
  if (strcmp(color, "orange") == 0) return LED_ORANGE;
  if (strcmp(color, "green")  == 0) return LED_GREEN;
  if (strcmp(color, "blue")   == 0) return LED_BLUE;
  if (strcmp(color, "red")    == 0) return LED_RED;
  return -1;
}

void LED_Setup() {
  pinMode(LED_ORANGE, OUTPUT);
  pinMode(LED_GREEN,  OUTPUT);
  pinMode(LED_BLUE,   OUTPUT);
  pinMode(LED_RED,    OUTPUT);

  digitalWrite(LED_ORANGE, LOW);
  digitalWrite(LED_GREEN,  LOW);
  digitalWrite(LED_BLUE,   LOW);
  digitalWrite(LED_RED,    LOW);

  digitalWrite(LED_ORANGE, HIGH); delay(100);
  digitalWrite(LED_GREEN,  HIGH); delay(100);
  digitalWrite(LED_BLUE,   HIGH); delay(100);
  digitalWrite(LED_RED,    HIGH); delay(100);

  digitalWrite(LED_ORANGE, LOW);
  digitalWrite(LED_GREEN,  LOW);
  digitalWrite(LED_BLUE,   LOW);
  digitalWrite(LED_RED,    LOW);
}

void LED(const char* action, const char* color) {
  int pin = _ledPin(color);
  if (pin == -1) return;
  if      (strcmp(action, "on")     == 0) digitalWrite(pin, HIGH);
  else if (strcmp(action, "off")    == 0) digitalWrite(pin, LOW);
  else if (strcmp(action, "toggle") == 0) digitalWrite(pin, !digitalRead(pin));
}

void LED_Number(float maxBit, int var) {
  if (var < 0) var = 0;
  bool orangeOn = var >= (maxBit / 1); if (orangeOn) var -= (maxBit / 1);
  bool greenOn  = var >= (maxBit / 2); if (greenOn)  var -= (maxBit / 2);
  bool blueOn   = var >= (maxBit / 4); if (blueOn)   var -= (maxBit / 4);
  bool redOn    = var >= (maxBit / 8);
  digitalWrite(LED_ORANGE, orangeOn ? HIGH : LOW);
  digitalWrite(LED_GREEN,  greenOn  ? HIGH : LOW);
  digitalWrite(LED_BLUE,   blueOn   ? HIGH : LOW);
  digitalWrite(LED_RED,    redOn    ? HIGH : LOW);
}
