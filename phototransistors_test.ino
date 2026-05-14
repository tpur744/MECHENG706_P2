// Phototransistor pins

//SECOND RIGHT WORKS _ BLUE GREEN
const int sensor1 = A8;
const int sensor2 = A9;
const int sensor3 = A10;
const int sensor4 = A11;

void setup() {
  Serial.begin(9600);

  // Set analogue pins as inputs
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
}

void loop() {
  int value1 = analogRead(sensor1);
  int value2 = analogRead(sensor2);
  int value3 = analogRead(sensor3);
  int value4 = analogRead(sensor4);

  Serial.print("A8 (LEFTMOST): ");
  Serial.print(value1);

  Serial.print("  A9 (SECOND LEFT): ");
  Serial.print(value2);

  Serial.print("  A10 (SECOND RIGHT): ");
  Serial.print(value3);

  Serial.print("  A11 (RIGHTMOST): ");
  Serial.println(value4);

  delay(100);
}