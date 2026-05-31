// =====================================================
// HEAD TO LIGHT — redesigned v3
//   Reactive obstacle avoidance (servo-snap + self-directed strafe)
//   merged into the v2 FSM. Strafe direction is chosen live from the
//   front IRs each tick; strafing continues until the obstacle clears,
//   then runs coast -> forward burst -> rescan -> rotate as before.
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
    integral    += error * dt;
    float derivative = (error - prevError) / dt;
    float out    = Kp * error + Ki * integral + Kd * derivative;
    prevError    = error;
    lastTime     = now;
    return out;
  }

  void reset() { integral = 0; prevError = 0; lastTime = millis(); }
};

PID pidForward;
PID pidStrafeR;
PID pidStrafeL;

// =====================================================
// PINS
// =====================================================
const int trigPin = 48, echoPin = 49;
const int fanPin  = 22;
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
int rot_bias = 0;
int rot_bias_mag = 100;
unsigned long rotBiasStart = 0;
const unsigned long ROT_BIAS_TIMEOUT_MS = 5000;

// =====================================================
// GYRO
// =====================================================
Adafruit_BNO08x   bno08x(-1);
sh2_SensorValue_t sensorValue;

float gyroZ = 0, heading = 0;
unsigned long lastGyroTime = 0;

// =====================================================
// IR — exponential moving average
// =====================================================
const float IR_alpha = 0.5;
float leftAvg = 0, rightAvg = 0, frontLeftAvg = 0, frontRightAvg = 0;

const float FRONT_IR_OBSTACLE_THRESHOLD = 1;

float getIRDistance(float avg, float a, float b) { return a / (avg - b); }
float getLeftDistance()  { return getIRDistance(leftAvg,  13.61, 0.469); }
float getRightDistance() { return getIRDistance(rightAvg, 17.85, 0.33);  }

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
  STRAFING,             // phase 1: reactive sideways move until front clear
  STRAFE_COAST,         // phase 2: wait POST_STRAFE_COAST_MS with front clear
  STRAFE_FORWARD,       // phase 3: short forward burst
  RESCAN_SWEEP,         // phase 4: rotate through ±60° arc
  RESCAN_ROTATE,        // phase 5: rotate to best heading found
  REVERSING,            // reverse after finding a light
  ALIGNING,             // servo alignment: minimise outer sensors (1+4)
  WAITING,              // fan on, wait up to 10s or until light goes out
  DONE
};
State state = SCANNING;

// ── LED patterns per state ─────────────────────────────────────────────────
void showStateOnLED(int s) {
  bool o = false, g = false, b = false, r = false;
  switch (s) {
    case SCANNING:             o=1;           break;  // orange
    case ROTATING:             o=1; g=1;      break;  // orange+green
    case DRIVING:                   g=1;      break;  // green
    //case STRAFING:                   b=1;     break;  // blue
    case STRAFE_COAST:              g=1; b=1; break;  // green+blue
    case STRAFE_FORWARD:            g=1;      break;  // green (moving)
    case RESCAN_SWEEP:         o=1;      r=1; break;  // orange+red
    case RESCAN_ROTATE:        o=1; g=1; b=1; break;  // orange+green+blue
    case REVERSING:                      r=1; break;  // red
    case WAITING:                   g=1; b=1; break;  // green+blue (fan on)
    case DONE:                 o=1; g=1; b=1; r=1; break; // all on
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
  int           dir            = 1;   // +1 right, -1 left (chosen live)
  float         targetHeading  = 0;
  unsigned long startTime      = 0;
  unsigned long coastStart     = 0;
  unsigned long forwardStart   = 0;
  unsigned long reverseStart   = 0;
  int           clearCount     = 0;   // consecutive clear ticks
};
StrafeState strafe;

const unsigned long STRAFE_PRE_REVERSE_MS   = 500;
const unsigned long STRAFE_PRE_FORWARD_MS   = 600;
const unsigned long STRAFE_DURATION_MS      = 4000;   // hard safety cap only
const unsigned long POST_STRAFE_COAST_MS    = 300;
const unsigned long STRAFE_FORWARD_BURST_MS = 250;

// =====================================================
// RESCAN STATE
// =====================================================
struct RescanState {
  float sweepStart       = 0;
  float sweepBestSum     = 0;
  float sweepBestHeading = 0;
  int   sweepDir         = 0;
  int   lastSum          = 0;
  bool  peakFound        = false;
  int   peakDropCount    = 0;
  bool  secondArc        = false;
};
RescanState rescan;

const float sweepRange = 60.0 * PI / 180.0;

// =====================================================
// REVERSE STATE
// =====================================================
unsigned long reverseStartTime = 0;

// =====================================================
// ALIGN STATE
// =====================================================
struct AlignState {
  int bestAngle    = centreAngle;
  int bestOuterSum = 99999;
  int sweepAngle   = centreAngle;
  int sweepDir     = -1;
};
AlignState alignSt;

// =====================================================
// WAITING STATE
// =====================================================
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

  pidForward.Kp = 175.0;  pidForward.Ki = 0.02;  pidForward.Kd = 5.0;
  pidStrafeR.Kp = 260.0; pidStrafeR.Ki = 0.005; pidStrafeR.Kd = 0.0;
  pidStrafeL.Kp = 500.0; pidStrafeL.Ki = 100.0; pidStrafeL.Kd = 100.0;

  if (!bno08x.begin_I2C()) { Serial.println("IMU FAIL"); while (1); }

  useGyroIntegration();
  delay(200);
  warmupIR();

  pidForward.reset();
  pidStrafeR.reset();
  pidStrafeL.reset();
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  updateGyro();
  updateIR();
  showStateOnLED(state);

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

    case DRIVING:             driveForward();  break;
    case STRAFING:             doStrafe();     break;
    case STRAFE_COAST:         doStrafe();     break;
    case STRAFE_FORWARD:       doStrafe();     break;
    case RESCAN_SWEEP:         doRescan();     break;
    case RESCAN_ROTATE:        doRescan();     break;

    case REVERSING:
      if (millis() - reverseStartTime < 500) {
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
// OBSTACLE HELPER
// =====================================================
bool frontIRObstacleDetected() {
  return (frontLeftAvg > FRONT_IR_OBSTACLE_THRESHOLD ||
          frontRightAvg > FRONT_IR_OBSTACLE_THRESHOLD);
}

// Returns -1 if obstacle is on the left, +1 if on the right, 0 if both/none.
int obstacleSideNow() {
  bool leftIR  = (frontLeftAvg  > FRONT_IR_OBSTACLE_THRESHOLD);
  bool rightIR = (frontRightAvg > FRONT_IR_OBSTACLE_THRESHOLD);
  if      (leftIR && !rightIR) return -1;
  else if (rightIR && !leftIR) return  1;
  return 0;  // both or neither
}

// Snap/sweep the scan servo toward whichever side is blocked. Mirrors the
// reactive servo logic from the old standalone obstacle sketch.
void servoTrackObstacle(bool obstacleDetected, int side) {
  unsigned long now = millis();
  unsigned long interval = obstacleDetected ? (servoDelay / 4) : servoDelay;
  if (now - lastServoTime < interval) return;
  lastServoTime = now;

  if (obstacleDetected && side != 0) {
    // Snap hard toward the blocked side
    servoAngle     = (side == -1) ? centreAngle + offsetAngle   // snap left
                                   : centreAngle - offsetAngle;  // snap right
    servoDirection = (side == -1) ? -1 : +1;
  } else if (obstacleDetected && side == 0) {
    // Both IRs triggered — hold position
  } else {
    // No obstacle — sweep normally
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
// STRAFE — reactive: chooses its own direction each tick from the
// front IRs, strafes until the path clears, then coasts/forwards.
// =====================================================
void doStrafe() {
  updateGyro();
  updateIR();
  unsigned long now = millis();

  bool leftIR  = (frontLeftAvg  > FRONT_IR_OBSTACLE_THRESHOLD);
  bool rightIR = (frontRightAvg > FRONT_IR_OBSTACLE_THRESHOLD);
  float dist   = getDistance();
  bool ultrasonicClose = (dist < 15.0);
  int  side    = obstacleSideNow();
  bool obstacleAhead = leftIR || rightIR || ultrasonicClose;

  // Keep the servo pointing at the obstacle while we manoeuvre
  servoTrackObstacle(obstacleAhead, side);

  // ── Phase 3: drive forward toward best heading ──────────────────────────
  if (state == STRAFE_FORWARD) {
    if (now - strafe.forwardStart >= STRAFE_FORWARD_BURST_MS) {
      stopMotors(); delay(200);

      // Return servo to centre before rescanning
      scanServo.write(centreAngle);
      servoAngle     = centreAngle;
      servoDirection = -1;
      delay(150);                 // give it time to physically arrive

      // rescan.sweepBestSum     = 0; redundant atm
      // rescan.sweepBestHeading = bestDriveHeading;
      // rescan.sweepStart       = heading;
      // rescan.peakDropCount    = 0;
      // rescan.sweepDir         = -strafe.dir;
      rot_bias     = -strafe.dir;   // strafed right(+1) → bias -1; left(-1) → +1
      rotBiasStart = millis();
      pidForward.reset(); pidStrafeR.reset(); pidStrafeL.reset();
      state = DRIVING;
    } else {
      move(speedVal, 0, 0);
    }
    return;
  }

  // ── Phase 2: coast ───────────────────────────────────────────────────────
  if (state == STRAFE_COAST) {
    if (obstacleAhead) {
      strafe.clearCount = 0;   // obstacle reappeared, go back to strafing
      state = STRAFING;
      return;
    }
    if (now - strafe.coastStart >= POST_STRAFE_COAST_MS) {
      stopMotors(); delay(100);
      strafe.forwardStart = millis();
      state = STRAFE_FORWARD;
      return;
    }
    int correction = (strafe.dir > 0)
      ? (int)pidStrafeR.compute(wrapAngle(strafe.targetHeading - heading))
      : (int)pidStrafeL.compute(wrapAngle(strafe.targetHeading - heading));
    move(0, strafe.dir * 1 * speedVal, 0 * correction);
    return;
  }

  // ── Phase 1: active strafing — pick direction live ───────────────────────
  // Decide which way to slide based on which side is blocked. If only one
  // front IR sees the obstacle, slide away from it. If both (or only the
  // ultrasonic) trip, fall back to comparing IR strengths.
  if (side == -1) {
    strafe.dir = 1;          // obstacle left  → strafe right
  } else if (side == 1) {
    strafe.dir = -1;         // obstacle right → strafe left


  } /*else if (obstacleAhead) {
    // Both/ultrasonic: compare strengths, slide toward the clearer side
    if      (frontLeftAvg > frontRightAvg) strafe.dir = 1;   // left stronger → go right
    else if (frontRightAvg > frontLeftAvg) strafe.dir = -1;  // right stronger → go left
    // else keep previous strafe.dir
  }*/

  if (obstacleAhead) {
    strafe.clearCount = 0;   // still blocked
  } else {
    strafe.clearCount++;
    if (strafe.clearCount >= 6) {  // ~200ms of clear readings at 20ms loop
      strafe.clearCount = 0;
      strafe.coastStart = now;
      state = STRAFE_COAST;
      return;
    }
  }

  // Hard safety cap so we can never strafe forever
  if (now - strafe.startTime >= STRAFE_DURATION_MS) {
    stopMotors(); delay(100);
    strafe.clearCount   = 0;
    strafe.forwardStart = millis();
    state = STRAFE_FORWARD;
    return;
  }

  // Directional LEDs: blue = strafing left, orange = strafing right
  if (strafe.dir > 0) {            // right
    digitalWrite(LED_ORANGE, HIGH);
    digitalWrite(LED_BLUE,   LOW);
  } else {                          // left
    digitalWrite(LED_BLUE,   HIGH);
    digitalWrite(LED_ORANGE, LOW);
  }
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED,   LOW);

  int correction = (strafe.dir > 0)
    ? (int)pidStrafeR.compute(wrapAngle(strafe.targetHeading - heading))
    : (int)pidStrafeL.compute(wrapAngle(strafe.targetHeading - heading));
  move(strafe.dir > 0 ? -100 : 0, strafe.dir * 1 * speedVal, 0 * correction);
}

// =====================================================
// RESCAN — rotate toward light, stop when sum peaks
// =====================================================
void doRescan() {
  rescan.sweepBestSum = bestDriveSum;  // snapshot before decay continues
  updateGyro();

  if (state == RESCAN_ROTATE) {
    if (rotateToHeading(rescan.sweepBestHeading)) {
      bestHeading      = bestDriveHeading = rescan.sweepBestHeading;
      bestDriveSum     = 0;
      servoLocked      = false;
      scanServo.write(centreAngle);
      servoAngle       = centreAngle + offsetAngle;
      servoDirection   = -1;
      state = DRIVING;
    }
    return;
  }

  int sum = analogRead(sensor1) + analogRead(sensor2)
          + analogRead(sensor3) + analogRead(sensor4);

  LED_Number(2000, sum);
  plotSensors();

  LED_Number(2000, sum);
  plotSensors();

  int target = rescan.sweepBestSum - 100;

  if (sum >= target && target > 0) {
    stopMotors(); delay(100);
    Serial.print("RESCAN: matched drive sum, heading=");
    Serial.println(heading);
    rescan.sweepBestHeading = heading;
    state = DRIVING;
    return;
  }

  float rotated = abs(wrapAngle(heading - rescan.sweepStart));
  if (rotated >= 2 * PI - 0.1f) {
    stopMotors(); delay(100);
    Serial.println("RESCAN: full rotation, falling back to full scan");
    scanForLight();
    rescan.sweepBestHeading = bestHeading;
    rescan.peakDropCount    = 0;
    state = DRIVING;
    return;
  }

  move(0, 0, -rescan.sweepDir * 150);
  delay(20);
}

// =====================================================
// TRIGGER STRAFE — entered from DRIVING when blocked.
//   Reactive method picks direction inside doStrafe(), so we just seed
//   an initial guess (compare IR strengths) and let the phase refine it.
// =====================================================
void triggerStrafe() {
  int side = obstacleSideNow();
  if      (side == -1) strafe.dir = 1;   // obstacle left  → strafe right
  else if (side == 1)  strafe.dir = -1;  // obstacle right → strafe left
  else strafe.dir = (frontLeftAvg >= frontRightAvg) ? 1 : -1;  // both: clearer side

  Serial.print("Obstacle! L_IR="); Serial.print(frontLeftAvg);
  Serial.print(" R_IR=");          Serial.print(frontRightAvg);
  Serial.print(" initialDir=");    Serial.println(strafe.dir);

  strafe.targetHeading = heading;
  strafe.startTime     = millis();
  strafe.clearCount    = 0;
  stopMotors(); delay(50);
  pidStrafeR.reset(); pidStrafeL.reset();
  state = STRAFING;
}

// =====================================================
// ALIGN — servo sweep to centre middle sensors on the light
// =====================================================
void doAlign() {
  int outerSum = min(analogRead(sensor2), analogRead(sensor3));
  LED_Number(250, outerSum);

  Serial.print("ALIGN angle="); Serial.print(alignSt.sweepAngle);
  Serial.print(" outer="); Serial.println(outerSum);

  if (outerSum > alignSt.bestOuterSum) {
    alignSt.bestOuterSum = outerSum;
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
  int sum = analogRead(sensor1) + analogRead(sensor2)
          + analogRead(sensor3) + analogRead(sensor4);
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
    if (millis() - scanStart > 6000) { Serial.println("SCAN TIMEOUT"); break; }
    if (totalRotated > (330.0 * PI / 180.0)) rotateSpeed = 80;

    move(0, 0, -rotateSpeed);

    int sum = analogRead(sensor1) + analogRead(sensor2)
            + analogRead(sensor3) + analogRead(sensor4);
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
  if (rot_bias != 0 && millis() - rotBiasStart >= ROT_BIAS_TIMEOUT_MS) {
    rot_bias = 0;
    stopMotors();
    delay(100);
    scanForLight();
    bestDriveHeading = bestHeading;
    bestDriveSum     = 0;
    pidForward.reset();
    state = ROTATING;   // rotate to the freshly found heading, then resume driving
    return;
  }

  float dist           = getDistance();
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

  int thr = 950;
  digitalWrite(LED_ORANGE, analogRead(sensor1) > thr ? HIGH : LOW);
  digitalWrite(LED_GREEN,  analogRead(sensor2) > thr ? HIGH : LOW);
  digitalWrite(LED_BLUE,   analogRead(sensor3) > thr ? HIGH : LOW);
  digitalWrite(LED_RED,    analogRead(sensor4) > thr ? HIGH : LOW);

  int  maxSensor   = max(max(analogRead(sensor1), analogRead(sensor2)),
                         max(analogRead(sensor3), analogRead(sensor4)));
  bool lightInView = (maxSensor > thr);
  bool frontIRClose    = frontIRObstacleDetected();
  bool ultrasonicClose = (dist < 15.0);

  if ((ultrasonicClose || frontIRClose) && !lightInView) {
    triggerStrafe();
    return;
  }

  int sum = analogRead(sensor1) + analogRead(sensor2)
          + analogRead(sensor3) + analogRead(sensor4);
  if ((sum > bestDriveSum) && (sum > 1000)) {
    bestDriveSum = sum;
    rot_bias = 0;
    float offsetRad  = (servoAngle - centreAngle) * (PI / 180.0);
    bestDriveHeading = wrapAngle(heading + offsetRad);
  }
  bestDriveSum = (int)(bestDriveSum * 0.995);

  int rotateCmd = (rot_bias != 0)
  ? rot_bias * rot_bias_mag
  : (int)pidForward.compute(wrapAngle(bestDriveHeading - heading));
  move(200, 0, rotateCmd);

  plotSensors();

  if (dist < 15.0 && lightInView) {
    stopMotors();
    alignSt.bestAngle    = centreAngle;
    alignSt.bestOuterSum = 99999;
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
  int v1 = analogRead(sensor1), v2 = analogRead(sensor2);
  int v3 = analogRead(sensor3), v4 = analogRead(sensor4);
  int sum = v1 + v2 + v3 + v4;
  Serial.print("S1:"); Serial.print(v1);
  Serial.print(" S2:"); Serial.print(v2);
  Serial.print(" S3:"); Serial.print(v3);
  Serial.print(" S4:"); Serial.print(v4);
  Serial.print(" SumNorm:"); Serial.print(map(sum, 0, 4092, 0, 1023));
  Serial.print(" Heading:"); Serial.println(
    map((int)(heading * 1000), (int)(-PI * 1000), (int)(PI * 1000), 0, 1023));
}

// =====================================================
// GYRO
// =====================================================
void updateGyro() {
  while (bno08x.getSensorEvent(&sensorValue)) {     // drain the queue
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
// IR
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
// MOTORS
// =====================================================
void move(int forward, int right, int rotate) {
  leftFrontMotor.writeMicroseconds(1500 + 1*(forward + right - rotate));
  leftRearMotor.writeMicroseconds(1500 + 1*(forward - right - rotate));
  rightRearMotor.writeMicroseconds(1500 - 1*(forward + right + rotate));
  rightFrontMotor.writeMicroseconds(1500 - 1*(forward - right + rotate));
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
const float DIST_alpha = 0.5;  // high = reactive, low = smoother

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
