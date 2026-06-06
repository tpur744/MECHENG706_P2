// =====================================================
// PT + ULTRASONIC TEST — smoothed, Serial Plotter output
//   Prints sensor values plus 3 boolean flags:
//     LightDetected — ≥ 2 phototransistors over 975
//     WithinRange   — ultrasonic distance < 13 cm
//     LightFound    — both above true
//   Flags are printed as 0 or 1000 so they show up on
//   the same plot scale as the PT values.
// =====================================================

// ── PHOTOTRANSISTOR PINS ─────────────────────────────
const int sensor1 = A8;
const int sensor2 = A9;
const int sensor3 = A10;
const int sensor4 = A11;

// ── ULTRASONIC PINS ──────────────────────────────────
const int trigPin = 48;
const int echoPin = 49;

// ── THRESHOLDS ───────────────────────────────────────
const int   PT_PAIR_THRESH   = 900;   // a sensor counts as "high" above this
const float RANGE_THRESH_CM  = 13.0;  // ultrasonic in-range cutoff

// ── PT EMA ───────────────────────────────────────────
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

void warmupPT() {
  for (int i = 0; i < 50; i++) { updatePT(); delay(4); }
}

// ── ULTRASONIC EMA ───────────────────────────────────
float distAvg = 0;
const float DIST_alpha = 0.5;

float getRawDistance() {
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long us = pulseIn(echoPin, HIGH, 30000UL);  // 30 ms timeout (~5 m)
  if (us == 0) return 999.0;                  // no echo → far
  return (us * 0.0343) / 2.0;
}

float getDistance() {
  distAvg = DIST_alpha * getRawDistance() + (1 - DIST_alpha) * distAvg;
  return distAvg;
}

void warmupDist() {
  for (int i = 0; i < 10; i++) { getDistance(); delay(20); }
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);

  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  warmupPT();
  warmupDist();
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  updatePT();
  float dist = getDistance();

  // ── Conditions ─────────────────────────────────────
  int highCount = (s1 > PT_PAIR_THRESH)
                + (s2 > PT_PAIR_THRESH)
                + (s3 > PT_PAIR_THRESH)
                + (s4 > PT_PAIR_THRESH);
  bool lightDetected = (highCount >= 2);
  bool withinRange   = (dist < RANGE_THRESH_CM);
  bool lightFound    = lightDetected && withinRange;

  // ── Serial Plotter line ────────────────────────────
  // Booleans scaled to 1000 so they're visible alongside PT values.
  Serial.print("S1:");             Serial.print((int)s1);
  Serial.print(" S2:");            Serial.print((int)s2);
  Serial.print(" S3:");            Serial.print((int)s3);
  Serial.print(" S4:");            Serial.print((int)s4);
  Serial.print(" Dist_cm:");       Serial.print(dist, 1);
  Serial.print(" LightDetected:"); Serial.print(lightDetected ? 1000 : 0);
  Serial.print(" WithinRange:");   Serial.print(withinRange   ? 1000 : 0);
  Serial.print(" LightFound:");    Serial.println(lightFound  ? 1000 : 0);

  delay(20);  // ~50 Hz
}
