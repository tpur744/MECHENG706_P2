int fanPin = 4;

void setup() {
  pinMode(fanPin, OUTPUT);
}

void loop() {
  digitalWrite(fanPin, HIGH);
  delay(3000);
  digitalWrite(fanPin, LOW);
  delay(3000);
}
