#include <Servo.h>
#include <Adafruit_BNO08x.h>


//digitalWrite(LED_PIN, LOW);   // off

// ================= MOTORS =================
// ─── Pin Definitions ───────────────────────────────────────────────────────────
#define LED_ORANGE  10
#define LED_GREEN   11
#define LED_BLUE    12
#define LED_RED     13

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
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;

float gyroZ = 0;
float heading = 0;
unsigned long lastGyroTime = 0;

// ================= PHOTOTRANSISTORS =================
const int sensor1 = A8;   // LEFTMOST
const int sensor2 = A9;   // SECOND LEFT
const int sensor3 = A10;  // SECOND RIGHT
const int sensor4 = A11;  // RIGHTMOST

// ================= SCAN RESULTS =================
float bestHeading = 0;
int   bestSum     = 0;

// ================= FORWARD PID =================
float Kp_fwd = 25.0;
float Ki_fwd = 0;
float Kd_fwd = 0;

float error_fwd = 0, prevError_fwd = 0, integral_fwd = 0;
unsigned long lastTime_fwd = 0;

// ================= ALIGN PID (rotate-to-heading) =================
float Kp_align = 200.0;
float Ki_align = 5;
float Kd_align = 0.0;

float alignPrevError = 0;
float alignIntegral  = 0;
unsigned long alignLastTime = 0;

// ================= STATE MACHINE =================
enum State { SCANNING, ROTATING, DRIVING, DONE };
State state = SCANNING;

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  LED_Setup();

  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);

  leftFrontMotor.attach(left_front);
  leftRearMotor.attach(left_rear);
  rightRearMotor.attach(right_rear);
  rightFrontMotor.attach(right_front);

  stopMotors();

  if (!bno08x.begin_I2C()) {
    Serial.println("IMU FAIL");
    while (1);
  }

  useGyroIntegration();
  delay(200);

  lastTime_fwd    = millis();
  alignLastTime   = millis();
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  updateGyro();

  switch (state) {

    // --------------------------------------------------
    case SCANNING:
      scanForLight();
      stopMotors();
      delay(300);
      // Reset forward PID clean before driving
      error_fwd = 0; prevError_fwd = 0; integral_fwd = 0;
      lastTime_fwd = millis();
      alignIntegral = 0; alignPrevError = 0;
      alignLastTime = millis();
      state = ROTATING;
      break;

    // --------------------------------------------------
    case ROTATING:
      if (rotateToHeading(bestHeading)) {
        delay(200);
        state = DRIVING;
      }
      break;

    // --------------------------------------------------
    case DRIVING:
      driveForward();
      break;

    // --------------------------------------------------
    case DONE:
      stopMotors();
      plotSensors();   // keep plotter alive so it doesn't freeze
      delay(20);
      break;
  }
}

// =====================================================
// PLOT HELPER — same 6-trace format used everywhere
// SumNorm and Heading mapped to 0-1023 so Y-axis stays 0-1023
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
// PHASE 1 — 360 scan, record heading of highest sum
// =====================================================
void scanForLight() {
  int rotateSpeed = 220;
  bestSum     = 0;
  bestHeading = heading;

  float lastHeading  = heading;
  float totalRotated = 0.0;

  while (totalRotated < 2 * PI) {
    updateGyro();

    if (totalRotated > (330.0 * PI / 180.0)) rotateSpeed = 80;

    // Negative so the FRONT of the robot sweeps through the scan
    move(0, 0, -rotateSpeed);

    int v1  = analogRead(sensor1);
    int v2  = analogRead(sensor2);
    int v3  = analogRead(sensor3);
    int v4  = analogRead(sensor4);
    int sum = v1 + v2 + v3 + v4;
    LED_Number(2000, sum);

    // Keep the heading where sensors were brightest
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
    totalRotated += abs(delta);
    lastHeading = heading;

    delay(20);
  }

  stopMotors();

  // Sensors face the opposite direction to the robot's forward,
  // so the recorded peak heading is 180° from where we actually want to go.
  bestHeading;
  while (bestHeading >  PI) bestHeading -= 2 * PI;
  while (bestHeading < -PI) bestHeading += 2 * PI;
}

// =====================================================
// PHASE 2 — Rotate to bestHeading using align PID
// Returns true once within 2 degrees
// =====================================================
bool rotateToHeading(float target) {
  updateGyro();

  float headingError = target - heading;
  // while (headingError >  2*PI) headingError -= 2 * PI;
  // while (headingError < 2*-PI) headingError += 2 * PI;

  plotSensors();

  float absErr = abs(headingError);
  int dir = (headingError > 0) ? 1 : -1;
  LED_Number(128,absErr);
  if (headingError > 0.01){
    LED("on", "red");
  }
  LED_Number(0.1, absErr);


  // ---- Deadband: close enough, stop ----
  if (absErr < (3.0 * PI / 180.0)) {          // within ±3°
    stopMotors();
    return true;
  }

  // ---- Two-speed: slow down inside the last 20° to avoid overshoot ----
  int rotateSpeed;
  if (absErr > (30.0 * PI / 180.0)) {
    rotateSpeed = 250;   // slow — just enough to keep moving
    move(0,0, rotateSpeed);
  } else {
    move(0, 0, 100);
  }
  delay(20);
  return false;
}

// =====================================================
// PHASE 3 — Drive forward, maintaining bestHeading
// Add your own stop condition in the marked section
// =====================================================
void driveForward() {
  updateGyro();

  float correction = computePIDForward(bestHeading, heading);
  move(250, 0, (int)correction);

  plotSensors();

  // ---- Add your stop condition here ----
  // Example — stop when sensors go dark again (light passed):
  // int sum = analogRead(sensor1) + analogRead(sensor2)
  //         + analogRead(sensor3) + analogRead(sensor4);
  // if (sum < bestSum * 0.3) { state = DONE; }
  //
  // Example — ultrasonic stop:
  // if (getDistance() < 15.0) { state = DONE; }

  delay(20);
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
// ALIGN PID (used for rotating to bestHeading)
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
// MOTOR CONTROL
// =====================================================
int clampMotor(int val) {
  // if (val < 1250) val = 1250;
  // if (val > 1750) val = 1750;
  // if (val > (1500 - clampVal) && val < 1500) val = 1500 - clampVal;
  // if (val < (1500 + clampVal) && val > 1500) val = 1500 + clampVal;
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

// ─── Internal Helper ───────────────────────────────────────────────────────────
static int _ledPin(const char* color) {
  if (strcmp(color, "orange") == 0) return LED_ORANGE;
  if (strcmp(color, "green")  == 0) return LED_GREEN;
  if (strcmp(color, "blue")   == 0) return LED_BLUE;
  if (strcmp(color, "red")    == 0) return LED_RED;
  return -1; // unknown color
}

// ─── LED Setup & Startup Sequence ──────────────────────────────────────────────
void LED_Setup() {
  pinMode(LED_ORANGE, OUTPUT);
  pinMode(LED_GREEN,  OUTPUT);
  pinMode(LED_BLUE,   OUTPUT);
  pinMode(LED_RED,    OUTPUT);

  // All off initially
  digitalWrite(LED_ORANGE, LOW);
  digitalWrite(LED_GREEN,  LOW);
  digitalWrite(LED_BLUE,   LOW);
  digitalWrite(LED_RED,    LOW);

  // Startup sequence: cascade on one by one, each 100ms apart
  digitalWrite(LED_ORANGE, HIGH); delay(100);
  digitalWrite(LED_GREEN,  HIGH); delay(100);
  digitalWrite(LED_BLUE,   HIGH); delay(100);
  digitalWrite(LED_RED,    HIGH); delay(100);

  // All off after sequence
  digitalWrite(LED_ORANGE, LOW);
  digitalWrite(LED_GREEN,  LOW);
  digitalWrite(LED_BLUE,   LOW);
  digitalWrite(LED_RED,    LOW);
}

// ─── LED Control ───────────────────────────────────────────────────────────────
// Usage: LED("on", "green")  |  LED("off", "red")  |  LED("toggle", "blue")
void LED(const char* action, const char* color) {
  int pin = _ledPin(color);
  if (pin == -1) return; // invalid color, do nothing

  if (strcmp(action, "on") == 0) {
    digitalWrite(pin, HIGH);
  } else if (strcmp(action, "off") == 0) {
    digitalWrite(pin, LOW);
  } else if (strcmp(action, "toggle") == 0) {
    digitalWrite(pin, !digitalRead(pin));
  }
}

// ─── LED Number Display ────────────────────────────────────────────────────────
// Displays a value as a 4-bit binary number across the 4 LEDs.
// MSB = Yellow (bit3/8), Green (bit2/4), Blue (bit1/2), LSB = Red (bit0/1)
// The value is floor-divided into the largest expressible 4-bit representation.
// e.g. LED_Number(128, 96)  -> binary 0110 -> Green ON, Blue ON
//      LED_Number(128, 97)  -> same (97 floor = 96 in this scale)
//      LED_Number(128, 255) -> binary 1111 -> all ON
//      LED_Number(128, 0)   -> binary 0000 -> all OFF
void LED_Number(float maxBit, int var) {
  if (var < 0)      var = 0;

  bool orangeOn = var >= (maxBit / 1);  if (orangeOn) var -= (maxBit / 1);
  bool greenOn  = var >= (maxBit / 2);  if (greenOn)  var -= (maxBit / 2);
  bool blueOn   = var >= (maxBit / 4);  if (blueOn)   var -= (maxBit / 4);
  bool redOn    = var >= (maxBit / 8);

  digitalWrite(LED_ORANGE, orangeOn ? HIGH : LOW);
  digitalWrite(LED_GREEN,  greenOn  ? HIGH : LOW);
  digitalWrite(LED_BLUE,   blueOn   ? HIGH : LOW);
  digitalWrite(LED_RED,    redOn    ? HIGH : LOW);
}
