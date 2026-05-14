#include <Servo.h>

Servo myServo;

const int servoPin = 9;

// Centre position (treated as "0")
const int centreAngle = 120;

// Oscillation range
const int offsetAngle = 45;

// Motion settings
const int stepDelay = 20;   // Smaller = faster
const int stepSize = 1;     // Degrees per step

void setup() {
  myServo.attach(servoPin);

  // Initialise at centre position
  myServo.write(centreAngle);
  delay(0);
}

void loop() {

  // Move from centre +45 down to centre -45
  for (int angle = centreAngle + offsetAngle;
       angle >= centreAngle - offsetAngle;
       angle -= stepSize) {

    myServo.write(angle);
    delay(stepDelay);
  }

  // Move back from centre -45 to centre +45
  for (int angle = centreAngle - offsetAngle;
       angle <= centreAngle + offsetAngle;
       angle += stepSize) {

    myServo.write(angle);
    delay(stepDelay);
  }
}
