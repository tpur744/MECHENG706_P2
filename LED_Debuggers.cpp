//This code is for debugging using the onboard leds. im thinking like seeing if things are above thresholds or using the leds to show like a bit value


// Arduino ATmega2560 LED Debug Library
// Pins: 10=Yellow, 11=Green, 12=Blue, 13=Red

// ─── Pin Definitions ───────────────────────────────────────────────────────────
#define LED_ORANGE  10
#define LED_GREEN   11
#define LED_BLUE    12
#define LED_RED     13

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
void LED_Number(int maxBit, int var) {
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

// ─── Example Usage ─────────────────────────────────────────────────────────────
void setup() {
  LED_Setup(); // Init pins + run startup sequence
}

void loop() {
  LED_Number(64,97);
  delay(2000);
}
