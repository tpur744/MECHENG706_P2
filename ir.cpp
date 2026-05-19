// =====================================================
// IR SENSOR SYSTEM (EXTRACTED + COMBINED)
// =====================================================

#include <Arduino.h>

// =====================================================
// IR SENSOR PINS
// =====================================================

const int LEFT_IR       = A12;
const int RIGHT_IR      = A13;
const int BACK_LEFT_IR  = A14;
const int BACK_RIGHT_IR = A15;

// =====================================================
// SETTINGS
// =====================================================

// EMA smoothing factor
const float IR_alpha = 0.02;

// Number of samples for averaging
const int IR_SAMPLE_AVG = 20;

// =====================================================
// SENSOR FILTER STORAGE
// =====================================================

float leftAvg      = 0;
float rightAvg     = 0;
float backleftAvg  = 0;
float backrightAvg = 0;

// =====================================================
// REAR IR CALIBRATION COEFFICIENTS
// =====================================================

float backLeftCoeffs[2]  = { 13.69, 0.31 };
float backRightCoeffs[2] = { 18.68, 0.21 };

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

  pinMode(LEFT_IR, INPUT);
  pinMode(RIGHT_IR, INPUT);
  pinMode(BACK_LEFT_IR, INPUT);
  pinMode(BACK_RIGHT_IR, INPUT);

  // Warm up EMA filters
  warmupIR();
}

// =====================================================
// LOOP
// =====================================================

void loop() {

  updateIR();

  float leftDist  = getLeftDistance();
  float rightDist = getRightDistance();
  float rearDist  = getRearIRDistance();

  Serial.print("Left IR: ");
  Serial.print(leftDist);
  Serial.print(" cm   ");

  Serial.print("Right IR: ");
  Serial.print(rightDist);
  Serial.print(" cm   ");

  Serial.print("Rear IR: ");
  Serial.print(rearDist);
  Serial.println(" cm");

  delay(100);
}

// =====================================================
// UPDATE IR FILTERS
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

// =====================================================
// WARMUP FILTERS
// =====================================================

void warmupIR() {

  for (int i = 0; i < 50; i++) {
    updateIR();
    delay(4);
  }
}

// =====================================================
// LEFT IR DISTANCE
// =====================================================

float getLeftDistance() {

  return 13.61 / (leftAvg - 0.469);
}

// =====================================================
// RIGHT IR DISTANCE
// =====================================================

float getRightDistance() {

  return 17.85 / (rightAvg - 0.33);
}

// =====================================================
// READ RAW IR VOLTAGE
// =====================================================

float readIRVoltage(int pin) {

  unsigned long sum = 0;

  for (int i = 0; i < IR_SAMPLE_AVG; i++) {

    sum += analogRead(pin);

    delayMicroseconds(200);
  }

  return (sum / (float)IR_SAMPLE_AVG) * (5.0 / 1024.0);
}

// =====================================================
// CONVERT VOLTAGE TO DISTANCE
// =====================================================

float irVoltageToDistance(float v, float a, float b) {

  if (v <= 1.0) return -1.0;

  float ratio = v / b;

  return a / log(ratio);
}

// =====================================================
// REAR IR DISTANCE
// =====================================================

float getRearIRDistance() {

  float vLeft  = readIRVoltage(BACK_LEFT_IR);
  float vRight = readIRVoltage(BACK_RIGHT_IR);

  float dLeft =
      irVoltageToDistance(
          vLeft,
          backLeftCoeffs[0],
          backLeftCoeffs[1]);

  float dRight =
      irVoltageToDistance(
          vRight,
          backRightCoeffs[0],
          backRightCoeffs[1]);

  Serial.print("Rear Left: ");
  Serial.print(dLeft);

  Serial.print(" cm   Rear Right: ");
  Serial.print(dRight);

  Serial.println(" cm");

  bool leftValid  = (dLeft > 0);
  bool rightValid = (dRight > 0);

  if (leftValid && rightValid) {
    return (dLeft + dRight) / 2.0;
  }

  return -1.0;
}
