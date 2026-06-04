// =====================================================
// HEAD TO LIGHT — v7
//   Changes from v6:
//   - SIDE IR OVERRIDE during active strafing: if one
//     side IR is within ~10cm, force direction AWAY from
//     that wall and lock for the rest of the strafe.
//     Entry into strafing (triggerStrafe) is unchanged.
//   - Removed STRAFE_COAST (POST_STRAFE_COAST_MS was 0).
//   - Removed rot_bias machinery (never set non-zero).
//   - Removed unreachable post-drive RESCAN states.
//   - Removed strafe correction PIDs (output was *0).
//   - Removed unused constants and StrafeState fields.
// =====================================================

#include <Servo.h>
#include <Adafruit_BNO08x.h>

// =====================================================
// PID STRUCT
// =====================================================
struct PID {
  float Kp, Ki, Kd;
  float integral = 0, prevError = 0;
  unsigned long lastTime = 0;

  float compute(float error) {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    if (dt <= 0) return 0;
    integral         += error * dt;
    float derivative  = (error - prevError) / dt;
    float out         = Kp * error + Ki * integral + Kd * derivative;
    prevError         = error;
    lastTime          = now;
    return out;
  }

  void reset() { integral = 0; prevError = 0; lastTime = millis(); }
};

PID pidForward;

// =====================================================
// PINS
// =====================================================
const int trigPin = 48, echoPin = 49;
const int fanPin  = 4;
const int servoPin = 9;

const byte left_front  = 46, left_rear  = 47;
const byte right_rear  = 50, right_front = 51;

const int sensor1 = A8,  sensor2 = A9;
const int sensor3 = A10, sensor4 = A11;

const int LEFT_IR       = A12, RIGHT_IR       = A13;
const int FRONT_LEFT_IR = A14, FRONT_RIGHT_IR = A15;

#define LED_ORANGE 10
#define LED_GREEN  11
#define LED_BLUE   12
#define LED_RED    13

// =====================================================
// SERVO
// =====================================================
Servo scanServo;
const int centreAngle = 120, offsetAngle = 35, servoStep = 4, servoDelay = 20;

int  servoAngle     = centreAngle + offsetAngle;
int  servoDirection = -1;
bool servoLocked    = false;
unsigned long lastServoTime = 0;

// =====================================================
// MOTORS
// =====================================================
Servo leftFrontMotor, leftRearMotor, rightRearMotor, rightFrontMotor;
const int speedVal = 220;

// =====================================================
// GYRO
// =====================================================
Adafruit_BNO08x   bno08x(-1);
sh2_SensorValue_t sensorValue;

float gyroZ = 0, heading = 0;
unsigned long lastGyroTime = 0;

// =====================================================
// OBSTACLE IR — exponential moving average
// =====================================================
const float IR_alpha = 0.5;
float leftAvg = 0, rightAvg = 0, frontLeftAvg = 0, frontRightAvg = 0;

const float FRONT_IR_OBSTACLE_THRESHOLD = 1.7;
const float SIDE_IR_OBSTACLE_THRESHOLD  = 2.0;  // ~10cm with current calibration

float getIRDistance(float avg, float a, float b) { return a / (avg - b); }
float getLeftDistance()  { return getIRDistance(leftAvg,  13.61, 0.469); }
float getRightDistance() { return getIRDistance(rightAvg, 17.85, 0.33);  }

// =====================================================
// PHOTOTRANSISTORS — exponential moving average
//   Raw counts (0–1023), NOT converted to voltage.
// =====================================================
const float PT_alpha = 0.5;
float s1 = 0, s2 = 0, s3 = 0, s4 = 0;

void updatePT() {
  auto ema = [](int pin, float prev) {
    return PT_alpha * analogRead(pin) + (1 - PT_alpha) * prev;
  };
  s1 = ema(sensor1, s1);
  s2 = ema(sensor2, s2);
  s3 = ema(sensor3, s3);
  s4 = ema(sensor4, s4);
}

// =====================================================
// SCAN RESULTS / DRIVE TRACKING
// =====================================================
float bestHeading = 0, bestDriveHeading = 0;
int   bestSum = 0,     bestDriveSum = 0;

// =====================================================
// STATE MACHINE
// =====================================================
enum State {
  SCANNING,
  ROTATING,
  DRIVING,
  STRAFING,          // phase 1: reactive sideways move until front clear
  STRAFE_FORWARD,    // phase 2: short forward burst
  STRAFE_RESCAN,     // phase 3: rotate, read PTs, lock onto peak heading
  REVERSING,         // reverse after extinguishing a light
  ALIGNING,          // servo alignment: maximise inner sensors (s2+s3)
  WAITING,           // fan on, wait up to 10s or until light goes out
  DONE
};
State state = SCANNING;

// ── LED patterns per state ───────────────────────────────
void showStateOnLED(int s) {
  bool o = false, g = false, b = false, r = false;
  switch (s) {
    case SCANNING:        o=1;                break;  // orange
    case ROTATING:        o=1; g=1;           break;  // orange+green
    case DRIVING:              g=1;           break;  // green
    case STRAFE_FORWARD:       g=1;           break;  // green (moving)
    case STRAFE_RESCAN:   o=1;      b=1;      break;  // orange+blue
    case REVERSING:                       r=1; break; // red
    case WAITING:              g=1; b=1;      break;  // green+blue
    case DONE:            o=1; g=1; b=1; r=1; break;  // all on
  }
  digitalWrite(LED_ORANGE, o ? HIGH : LOW);
  digitalWrite(LED_GREEN,  g ? HIGH : LOW);
  digitalWrite(LED_BLUE,   b ? HIGH : LOW);
  digitalWrite(LED_RED,    r ? HIGH : LOW);
}

int  lightsFound = 0;
const int totalLights = 2;

// =====================================================
// STRAFE STATE
// =====================================================
struct StrafeState {
  int           dir            = 1;     // +1 right, -1 left
  bool          sideLocked     = false; // side IR has forced direction
  unsigned long startTime      = 0;
  unsigned long forwardStart   = 0;
  int           clearCount     = 0;     // consecutive clear ticks

  // STRAFE_RESCAN
  float rescanStartHeading = 0;
  float rescanBestHeading  = 0;
  int   rescanBestSum      = 0;
  int   rescanLastSum      = 0;
  int   rescanDropCount    = 0;
  int   rescanDir          = 1;
};
StrafeState strafe;

const unsigned long STRAFE_DURATION_MS      = 4000;  // hard safety cap
const unsigned long STRAFE_FORWARD_BURST_MS = 1000;

// =====================================================
// REVERSE / ALIGN / WAITING STATE
// =====================================================
unsigned long reverseStartTime = 0;

struct AlignState {
  int bestAngle    = centreAngle;
  int bestInnerSum = 0;
  int sweepAngle   = centreAngle;
  int sweepDir     = -1;
};
AlignState alignSt;

unsigned long waitStart = 0;
const unsigned long WAIT_MAX_MS     = 10000;
const int           WAIT_OFF_THRESH = 1000;

// =====================================================
// SETUP
// =====================================================
void setup() {
  pinMode(fanPin,  OUTPUT); digitalWrite(fanPin, LOW);
  pinMode(trigPin, OUTPUT); pinMode(echoPin, INPUT);

  Serial.begin(115200);
  LED_Setup();

  scanServo.attach(servoPin);
  scanServo.write(centreAngle);

  pinMode(sensor1, INPUT); pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT); pinMode(sensor4, INPUT);
  pinMode(LEFT_IR, INPUT); pinMode(RIGHT_IR, INPUT);
  pinMode(FRONT_LEFT_IR, INPUT); pinMode(FRONT_RIGHT_IR, INPUT);

  leftFrontMotor.attach(left_front);
  leftRearMotor.attach(left_rear);
  rightRearMotor.attach(right_rear);
  rightFrontMotor.attach(right_front);
  delay(100);
  stopMotors();

  pidForward.Kp = 125.0;  pidForward.Ki = 0.02;  pidForward.Kd = 5.0;

  if (!bno08x.begin_I2C()) { Serial.println("IMU FAIL"); while (1); }

  useGyroIntegration();
  delay(200);
  warmupIR();
  warmupPT();

  pidForward.reset();
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  updateGyro();
  updateIR();
  updatePT();
  //showStateOnLED(state);

  switch (state) {

    case SCANNING:
      scanForLight();
      stopMotors();
      delay(300);
      pidForward.reset();
      bestDriveHeading = bestHeading;
      bestDriveSum     = 0;
      state = ROTATING;
      break;

    case ROTATING:
      if (rotateToHeading(bestHeading)) {
        delay(200);
        state = DRIVING;
      }
      break;

    case DRIVING:        driveForward();   break;
    case STRAFING:       doStrafe();       break;
    case STRAFE_FORWARD: doStrafe();       break;
    case STRAFE_RESCAN:  doStrafeRescan(); break;

    case REVERSING:
      if (millis() - reverseStartTime < 300) {
        move(-150, 0, 0);
        plotSensors();
      } else {
        stopMotors();
        delay(300);
        bestSum = bestDriveSum = 0;
        pidForward.reset();
        servoLocked    = false;
        scanServo.write(centreAngle);
        servoAngle     = centreAngle + offsetAngle;
        servoDirection = -1;
        state = SCANNING;
      }
      break;

    case ALIGNING:  doAlign();   break;
    case WAITING:   doWaiting(); break;

    case DONE:
      stopMotors();
      plotSensors();
      delay(20);
      break;
  }
}

// =====================================================
// OBSTACLE HELPERS
// =====================================================
bool frontIRObstacleDetected() {
  return (frontLeftAvg  > FRONT_IR_OBSTACLE_THRESHOLD ||
          frontRightAvg > FRONT_IR_OBSTACLE_THRESHOLD);
}

bool sideIRObstacleDetected(int dir) {
  // dir: +1 = strafing right, check right IR; -1 = strafing left, check left IR
  if (dir > 0) return rightAvg > SIDE_IR_OBSTACLE_THRESHOLD;
  else         return leftAvg  > SIDE_IR_OBSTACLE_THRESHOLD;
}

// Returns -1 if obstacle on left, +1 if on right, 0 if both/none.
int obstacleSideNow() {
  bool leftIR  = (frontLeftAvg  > FRONT_IR_OBSTACLE_THRESHOLD);
  bool rightIR = (frontRightAvg > FRONT_IR_OBSTACLE_THRESHOLD);
  if      (leftIR && !rightIR) return -1;
  else if (rightIR && !leftIR) return  1;
  return 0;
}

// Snap/sweep the scan servo toward whichever side is blocked.
void servoTrackObstacle(bool obstacleDetected, int side) {
  unsigned long now = millis();
  unsigned long interval = obstacleDetected ? (servoDelay / 4) : servoDelay;
  if (now - lastServoTime < interval) return;
  lastServoTime = now;

  if (obstacleDetected && side != 0) {
    servoAngle     = (side == -1) ? centreAngle + offsetAngle
                                  : centreAngle - offsetAngle;
    servoDirection = (side == -1) ? -1 : +1;
  } else if (obstacleDetected && side == 0) {
    // Both IRs triggered — hold position
  } else {
    servoAngle += servoDirection * servoStep;
    if (servoAngle <= centreAngle - offsetAngle) {
      servoAngle     = centreAngle - offsetAngle;
      servoDirection = +1;
    } else if (servoAngle >= centreAngle + offsetAngle) {
      servoAngle     = centreAngle + offsetAngle;
      servoDirection = -1;
    }
  }
  scanServo.write(servoAngle);
}

// =====================================================
// STRAFE  (phases 1–2)
// =====================================================
void doStrafe() {
  updateGyro();
  updateIR();
  updatePT();
  unsigned long now = millis();

  bool leftIR  = (frontLeftAvg  > FRONT_IR_OBSTACLE_THRESHOLD);
  bool rightIR = (frontRightAvg > FRONT_IR_OBSTACLE_THRESHOLD);
  float dist   = getDistance();
  bool ultrasonicClose = (dist < 15.0);
  int  side    = obstacleSideNow();
  bool obstacleAhead = leftIR || rightIR || ultrasonicClose;

  servoTrackObstacle(obstacleAhead, side);

  // ── Phase 2: forward burst ──────────────────────────────
  if (state == STRAFE_FORWARD) {
    if (obstacleAhead) {
      // Re-trigger a fresh strafe: picks a new direction based
      // on which side the new obstacle is on, and resets the
      // side-IR lock.
      triggerStrafe();
      return;
    }
    if (now - strafe.forwardStart >= STRAFE_FORWARD_BURST_MS) {
      stopMotors(); delay(200);

      // Centre servo for a clean PT sweep
      scanServo.write(centreAngle);
      servoAngle     = centreAngle;
      servoDirection = -1;
      delay(150);

      // ── Hand off to PT-guided rescan ─────────────────
      strafe.rescanStartHeading = heading;
      strafe.rescanBestHeading  = heading;
      strafe.rescanBestSum      = 0;
      strafe.rescanLastSum      = 0;
      strafe.rescanDropCount    = 0;
      // Light is on the side we strafed AWAY from
      strafe.rescanDir          = -strafe.dir;
      pidForward.reset();
      state = STRAFE_RESCAN;
    } else {
      move(speedVal, 0, 0);
    }
    return;
  }

  // ── Phase 1: active strafing ────────────────────────────

  // Side IR override: if a side IR is close, force direction
  // AWAY from that wall and lock for the rest of the strafe.
  if (!strafe.sideLocked) {
    bool leftClose  = (leftAvg  > SIDE_IR_OBSTACLE_THRESHOLD);
    bool rightClose = (rightAvg > SIDE_IR_OBSTACLE_THRESHOLD);
    if (leftClose && !rightClose) {
      strafe.dir = 1;
      strafe.sideLocked = true;
      Serial.println("STRAFE: side IR override → right (locked)");
    } else if (rightClose && !leftClose) {
      strafe.dir = -1;
      strafe.sideLocked = true;
      Serial.println("STRAFE: side IR override → left (locked)");
    }
  }

  // Front IR-based direction — only if not locked by a side IR
  if (!strafe.sideLocked) {
    if      (side == -1) strafe.dir =  1;  // obstacle left  → strafe right
    else if (side ==  1) strafe.dir = -1;  // obstacle right → strafe left
  }

  // Abort only if the chosen direction's side IR is also close
  // (i.e. pinned between walls).
  if (sideIRObstacleDetected(strafe.dir)) {
    stopMotors(); delay(100);
    Serial.println("STRAFE: both sides blocked — aborting strafe");
    strafe.clearCount   = 0;
    strafe.forwardStart = millis();
    state = STRAFE_FORWARD;
    return;
  }

  // Track clear ticks → transition to forward burst
  if (obstacleAhead) {
    strafe.clearCount = 0;
  } else {
    strafe.clearCount++;
    if (strafe.clearCount >= 10) {
      strafe.clearCount   = 0;
      strafe.forwardStart = millis();
      state = STRAFE_FORWARD;
      return;
    }
  }

  // Hard safety cap
  if (now - strafe.startTime >= STRAFE_DURATION_MS) {
    stopMotors(); delay(100);
    strafe.clearCount   = 0;
    strafe.forwardStart = millis();
    state = STRAFE_FORWARD;
    return;
  }

  // Directional LEDs: orange = strafing right, blue = strafing left
  if (strafe.dir > 0) {
    digitalWrite(LED_ORANGE, HIGH); digitalWrite(LED_BLUE, LOW);
  } else {
    digitalWrite(LED_BLUE, HIGH);   digitalWrite(LED_ORANGE, LOW);
  }
  digitalWrite(LED_GREEN, LOW); digitalWrite(LED_RED, LOW);

  move(strafe.dir > 0 ? -100 : 0, strafe.dir * speedVal, 0);
}

// =====================================================
// STRAFE RESCAN — phase 3
//   Rotate slowly, read PTs each tick. Lock onto the
//   heading where PT sum peaks, then → ROTATING → DRIVING.
// =====================================================
void doStrafeRescan() {
  updateGyro();
  updatePT();

  int sum = (int)(s1 + s2 + s3 + s4);
  LED_Number(2000, sum);

  Serial.print("STRAFE_RESCAN sum="); Serial.print(sum);
  Serial.print(" best=");             Serial.print(strafe.rescanBestSum);
  Serial.print(" dropCount=");        Serial.print(strafe.rescanDropCount);
  Serial.print(" heading=");          Serial.println(heading);

  // Track rising peak
  if (sum > strafe.rescanBestSum) {
    strafe.rescanBestSum     = sum;
    strafe.rescanBestHeading = heading;
    strafe.rescanDropCount   = 0;
  } else if (strafe.rescanBestSum > 800 && sum < strafe.rescanBestSum - 100) {
    strafe.rescanDropCount++;
  }

  // Peak confirmed after 3 consecutive dropping ticks
    float swept = abs(wrapAngle(heading - strafe.rescanStartHeading));
    bool sweptEnough = swept > (20.0 * PI / 180.0);

    if (strafe.rescanDropCount >= 3 && sweptEnough) {
    stopMotors(); delay(100);
    Serial.print("STRAFE_RESCAN: peak locked, heading=");
    Serial.println(strafe.rescanBestHeading);
    bestHeading      = strafe.rescanBestHeading;
    bestDriveHeading = strafe.rescanBestHeading;
    bestDriveSum     = 0;
    servoLocked      = false;
    scanServo.write(centreAngle);
    servoAngle       = centreAngle + offsetAngle;
    servoDirection   = -1;
    pidForward.reset();
    state = ROTATING;
    return;
  }

  // Safety: swept > 180° with no clear peak → fall back to full scan
  //float swept = abs(wrapAngle(heading - strafe.rescanStartHeading));
  if (swept > (180.0 * PI / 180.0)) {
    stopMotors(); delay(100);
    Serial.println("STRAFE_RESCAN: 180deg swept, no peak — full scan");
    scanForLight();
    bestDriveHeading = bestHeading;
    bestDriveSum     = 0;
    servoLocked      = false;
    scanServo.write(centreAngle);
    servoAngle       = centreAngle + offsetAngle;
    servoDirection   = -1;
    pidForward.reset();
    state = ROTATING;
    return;
  }

  // Keep rotating slowly in the chosen direction
  move(0, 0, -strafe.rescanDir * 120);
  delay(20);
}

// =====================================================
// TRIGGER STRAFE
// =====================================================
void triggerStrafe() {
  int side = obstacleSideNow();
  if      (side == -1) strafe.dir = 1;
  else if (side ==  1) strafe.dir = -1;
  else strafe.dir = (frontLeftAvg >= frontRightAvg) ? 1 : -1;

  Serial.print("Obstacle! L_IR="); Serial.print(frontLeftAvg);
  Serial.print(" R_IR=");          Serial.print(frontRightAvg);
  Serial.print(" initialDir=");    Serial.println(strafe.dir);

  strafe.startTime  = millis();
  strafe.clearCount = 0;
  strafe.sideLocked = false;
  stopMotors(); delay(50);
  state = STRAFING;
}

// =====================================================
// ALIGN — servo sweep to maximise inner sensors (s2+s3)
// =====================================================
void doAlign() {
  updatePT();
  int innerSum = (int)(s2 + s3);
  LED_Number(500, innerSum);

  Serial.print("ALIGN angle="); Serial.print(alignSt.sweepAngle);
  Serial.print(" inner="); Serial.println(innerSum);

  if ((innerSum > alignSt.bestInnerSum) && (min(s2,s3) > 600)) {
    alignSt.bestInnerSum = innerSum;
    alignSt.bestAngle    = alignSt.sweepAngle;
  }

  alignSt.sweepAngle += alignSt.sweepDir * servoStep;
  scanServo.write(alignSt.sweepAngle);
  delay(servoDelay);

  if (alignSt.sweepAngle <= centreAngle - offsetAngle && alignSt.sweepDir == -1) {
    alignSt.sweepDir = 1;
  }
  else if (alignSt.sweepAngle >= centreAngle + offsetAngle && alignSt.sweepDir == 1) {
    Serial.print("ALIGN best="); Serial.println(alignSt.bestAngle);
    scanServo.write(alignSt.bestAngle);
    delay(200);
    stopMotors();
    delay(200);
    digitalWrite(fanPin, HIGH);
    waitStart = millis();
    state = WAITING;
  }
}

// =====================================================
// WAITING — fan on until light goes out or 10s elapses
// =====================================================
void doWaiting() {
  updatePT();
  int sum = (int)(s1 + s2 + s3 + s4);
  LED_Number(2000, sum);
  plotSensors();

  bool lightGone = (sum < WAIT_OFF_THRESH);
  bool timedOut  = (millis() - waitStart >= WAIT_MAX_MS);

  if (lightGone || timedOut) {
    digitalWrite(fanPin, LOW);
    Serial.println(lightGone ? "LIGHT OFF — moving on" : "WAIT TIMEOUT — moving on");
    lightsFound++;
    if (lightsFound >= totalLights) { state = DONE; return; }
    reverseStartTime = millis();
    scanServo.write(centreAngle);
    state = REVERSING;
  }
}

// =====================================================
// 360° SCAN
// =====================================================
void scanForLight() {
  int rotateSpeed = 220;
  bestSum = 0; bestHeading = heading;

  float lastHeading = heading, totalRotated = 0.0;
  unsigned long scanStart = millis();

  while (totalRotated < 2 * PI) {
    updateGyro();
    updatePT();
    if (millis() - scanStart > 6000) { Serial.println("SCAN TIMEOUT"); break; }
    if (totalRotated > (330.0 * PI / 180.0)) rotateSpeed = 80;

    move(0, 0, -rotateSpeed);

    int sum = (int)(s1 + s2 + s3 + s4);
    LED_Number(2000, sum);

    if (sum > bestSum) { bestSum = sum; bestHeading = heading; }

    plotSensors();

    float delta = wrapAngle(heading - lastHeading);
    if (abs(delta) > 0.001) totalRotated += abs(delta);
    lastHeading = heading;
    delay(20);
  }

  stopMotors();
  bestHeading = wrapAngle(bestHeading);
}

// =====================================================
// ROTATE TO HEADING
// =====================================================
bool rotateToHeading(float target) {
  updateGyro();
  float err = wrapAngle(target - heading);

  plotSensors();
  LED_Number(0.1, abs(err));

  if (abs(err) < (3.0 * PI / 180.0)) { stopMotors(); return true; }

  int rotateSpeed = (abs(err) > (30.0 * PI / 180.0)) ? 250 : 100;
  move(0, 0, -(err > 0 ? 1 : -1) * rotateSpeed);
  delay(20);
  return false;
}

// =====================================================
// DRIVE FORWARD
// =====================================================
void driveForward() {
  updateGyro();
  updatePT();

  float dist            = getDistance();
  int   angleFromCentre = servoAngle - centreAngle;

  if (!servoLocked && dist < 0 && abs(angleFromCentre) < 3) {
    servoLocked = true;
    scanServo.write(centreAngle);
  }
  if (!servoLocked) {
    unsigned long now = millis();
    if (now - lastServoTime >= servoDelay) {
      lastServoTime = now;
      servoAngle += servoDirection * servoStep;
      if      (servoAngle <= centreAngle - offsetAngle) { servoAngle = centreAngle - offsetAngle; servoDirection = +1; }
      else if (servoAngle >= centreAngle + offsetAngle) { servoAngle = centreAngle + offsetAngle; servoDirection = -1; }
      scanServo.write(servoAngle);
    }
  }

  const int thr = 900;
  digitalWrite(LED_ORANGE, s1 > thr ? HIGH : LOW);
  digitalWrite(LED_GREEN,  s2 > thr ? HIGH : LOW);
  digitalWrite(LED_BLUE,   s3 > thr ? HIGH : LOW);
  digitalWrite(LED_RED,    s4 > thr ? HIGH : LOW);

  bool mid_lights_high     = (s2 + s3) > 950;
  if (mid_lights_high){
    LED("toggle", "orange");
  }
  bool outer_lights_lowish = (s1 < 500) || (s4 < 500);
  if (outer_lights_lowish){
    LED("toggle", "red");
  }

  bool pairHigh = ((s1 > thr) + (s2 > thr) + (s3 > thr) + (s4 > thr)) >= 2;
  //bool lightInView         = mid_lights_high && outer_lights_lowish;
  bool lightInView = pairHigh;
  bool frontIRClose        = frontIRObstacleDetected();
  bool ultrasonicClose     = (dist < 13.0);

  if ((ultrasonicClose || frontIRClose) && !lightInView) {
    triggerStrafe();
    return;
  }

  int sum = (int)(s1 + s2 + s3 + s4);
  if ((sum > bestDriveSum) && (sum > 1000)) {
    bestDriveSum = sum;
    float offsetRad  = (servoAngle - centreAngle) * (PI / 180.0);
    bestDriveHeading = wrapAngle(heading + offsetRad);
  }
  bestDriveSum = (int)(bestDriveSum * 0.995);

  int rotateCmd = (int)pidForward.compute(wrapAngle(bestDriveHeading - heading));
  move(200, 0, rotateCmd);

  plotSensors();

  if (dist < 13.0 && lightInView) {
    stopMotors();
    alignSt.bestAngle    = centreAngle;
    alignSt.bestInnerSum = 0;
    alignSt.sweepAngle   = centreAngle;
    alignSt.sweepDir     = -1;
    scanServo.write(centreAngle);
    delay(100);
    state = ALIGNING;
  }
}

// =====================================================
// HELPERS
// =====================================================
float wrapAngle(float a) {
  while (a >  PI) a -= 2 * PI;
  while (a < -PI) a += 2 * PI;
  return a;
}

void plotSensors() {
  int sum = (int)(s1 + s2 + s3 + s4);
  Serial.print("S1:"); Serial.print((int)s1);
  Serial.print(" S2:"); Serial.print((int)s2);
  Serial.print(" S3:"); Serial.print((int)s3);
  Serial.print(" S4:"); Serial.print((int)s4);
  Serial.print(" SumNorm:"); Serial.print(map(sum, 0, 4092, 0, 1023));
  Serial.print(" Heading:"); Serial.println(
    map((int)(heading * 1000), (int)(-PI * 1000), (int)(PI * 1000), 0, 1023));
}

// =====================================================
// GYRO
// =====================================================
void updateGyro() {
  while (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_GYROSCOPE_UNCALIBRATED) {
      gyroZ = sensorValue.un.gyroscope.z;
      unsigned long now = millis();
      heading = wrapAngle(heading - gyroZ * ((now - lastGyroTime) / 1000.0));
      lastGyroTime = now;
    }
  }
}

void useGyroIntegration() {
  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 0); delay(10);
  bno08x.enableReport(SH2_GYROSCOPE_UNCALIBRATED);  delay(50);
  lastGyroTime = millis(); heading = 0;
}

// =====================================================
// OBSTACLE IR
// =====================================================
void updateIR() {
  auto ema = [](int pin, float prev) {
    return IR_alpha * (analogRead(pin) * (5.0 / 1024.0)) + (1 - IR_alpha) * prev;
  };
  leftAvg       = ema(LEFT_IR,        leftAvg);
  rightAvg      = ema(RIGHT_IR,       rightAvg);
  frontLeftAvg  = ema(FRONT_LEFT_IR,  frontLeftAvg);
  frontRightAvg = ema(FRONT_RIGHT_IR, frontRightAvg);
}

void warmupIR() { for (int i = 0; i < 50; i++) { updateIR(); delay(4); } }

// =====================================================
// PHOTOTRANSISTORS
// =====================================================
void warmupPT() { for (int i = 0; i < 50; i++) { updatePT(); delay(4); } }

// =====================================================
// MOTORS
// =====================================================
void move(int forward, int right, int rotate) {
  leftFrontMotor.writeMicroseconds(1500 + (forward + right - rotate));
  leftRearMotor.writeMicroseconds(1500 + (forward - right - rotate));
  rightRearMotor.writeMicroseconds(1500 - (forward + right + rotate));
  rightFrontMotor.writeMicroseconds(1500 - (forward - right + rotate));
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
float distAvg = 0;
const float DIST_alpha = 0.5;

float getDistance() {
  distAvg = DIST_alpha * getRawDistance() + (1 - DIST_alpha) * distAvg;
  return distAvg;
}

float getRawDistance() {
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return (pulseIn(echoPin, HIGH) * 0.0343) / 2.0;
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
  int pins[] = { LED_ORANGE, LED_GREEN, LED_BLUE, LED_RED };
  for (int p : pins) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  for (int p : pins) { digitalWrite(p, HIGH); delay(100); }
  for (int p : pins) { digitalWrite(p, LOW); }
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
  bool o = var >= (maxBit / 1); if (o) var -= (maxBit / 1);
  bool g = var >= (maxBit / 2); if (g) var -= (maxBit / 2);
  bool b = var >= (maxBit / 4); if (b) var -= (maxBit / 4);
  bool r = var >= (maxBit / 8);
  digitalWrite(LED_ORANGE, o ? HIGH : LOW);
  digitalWrite(LED_GREEN,  g ? HIGH : LOW);
  digitalWrite(LED_BLUE,   b ? HIGH : LOW);
  digitalWrite(LED_RED,    r ? HIGH : LOW);
}
