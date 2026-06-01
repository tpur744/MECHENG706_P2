// ─── Pin Definitions ───────────────────────────────────────────────────────────
#define LED_ORANGE  10
#define LED_GREEN   11
#define LED_BLUE    12
#define LED_RED     13

// Phototransistor pins
const int sensor1 = A8;   // LEFTMOST
const int sensor2 = A9;   // SECOND LEFT
const int sensor3 = A10;  // SECOND RIGHT
const int sensor4 = A11;  // RIGHTMOST

// Threshold config
#define THRESHOLD     950
#define THRESHOLD_TOL 5
#define THRESHOLD_LOW  (THRESHOLD - THRESHOLD_TOL)   // 880
#define THRESHOLD_HIGH (THRESHOLD + THRESHOLD_TOL)   // 920

// Flash timing
#define FLASH_INTERVAL 500  // ms

// ─── Internal Helper ───────────────────────────────────────────────────────────
static int _ledPin(const char* color) {
  if (strcmp(color, "orange") == 0) return LED_ORANGE;
  if (strcmp(color, "green")  == 0) return LED_GREEN;
  if (strcmp(color, "blue")   == 0) return LED_BLUE;
  if (strcmp(color, "red")    == 0) return LED_RED;
  return -1;
}

// ─── LED Setup & Startup Sequence ──────────────────────────────────────────────
void LED_Setup() {
  pinMode(LED_ORANGE, OUTPUT);
  pinMode(LED_GREEN,  OUTPUT);
  pinMode(LED_BLUE,   OUTPUT);
  pinMode(LED_RED,    OUTPUT);

  digitalWrite(LED_ORANGE, LOW);
  digitalWrite(LED_GREEN,  LOW);
  digitalWrite(LED_BLUE,   LOW);
  digitalWrite(LED_RED,    LOW);

  // Startup sequence
  digitalWrite(LED_ORANGE, HIGH); delay(100);
  digitalWrite(LED_GREEN,  HIGH); delay(100);
  digitalWrite(LED_BLUE,   HIGH); delay(100);
  digitalWrite(LED_RED,    HIGH); delay(100);

  digitalWrite(LED_ORANGE, LOW);
  digitalWrite(LED_GREEN,  LOW);
  digitalWrite(LED_BLUE,   LOW);
  digitalWrite(LED_RED,    LOW);
}

// ─── LED Control ───────────────────────────────────────────────────────────────
void LED(const char* action, const char* color) {
  int pin = _ledPin(color);
  if (pin == -1) return;

  if (strcmp(action, "on") == 0) {
    digitalWrite(pin, HIGH);
  } else if (strcmp(action, "off") == 0) {
    digitalWrite(pin, LOW);
  } else if (strcmp(action, "toggle") == 0) {
    digitalWrite(pin, !digitalRead(pin));
  }
}

// ─── All LEDs On/Off ───────────────────────────────────────────────────────────
void LED_AllSet(bool state) {
  digitalWrite(LED_ORANGE, state ? HIGH : LOW);
  digitalWrite(LED_GREEN,  state ? HIGH : LOW);
  digitalWrite(LED_BLUE,   state ? HIGH : LOW);
  digitalWrite(LED_RED,    state ? HIGH : LOW);
}

// ─── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);

  LED_Setup();
}

// ─── Loop ──────────────────────────────────────────────────────────────────────
unsigned long lastFlashTime = 0;
bool flashState = false;

void loop() {
  float threshold = 900;
  digitalWrite(LED_ORANGE, analogRead(sensor1) > threshold ? HIGH : LOW);
  digitalWrite(LED_GREEN,  analogRead(sensor2) > threshold ? HIGH : LOW);
  digitalWrite(LED_BLUE,   analogRead(sensor3) > threshold ? HIGH : LOW);
  digitalWrite(LED_RED,    analogRead(sensor4) > threshold ? HIGH : LOW);
}
