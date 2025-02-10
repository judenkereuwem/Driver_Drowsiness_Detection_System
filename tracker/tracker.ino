#include <Servo.h>

Servo panServo;
Servo tiltServo;

int panPin = 6;   // Connect pan servo to pin 6
int tiltPin = 11; // Connect tilt servo to pin 11

int panAngle = 90; // Default angle for pan
int tiltAngle = 40; // Default angle for tilt

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 baud
  panServo.attach(panPin);
  tiltServo.attach(tiltPin);
  
  panServo.write(panAngle);  // Start at 90 degrees (centered)
  tiltServo.write(tiltAngle);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');  // Read until newline character
    if (data.startsWith("P")) {
      panAngle = data.substring(1).toInt();  // Get pan angle
      panServo.write(panAngle);              // Move pan servo
    } else if (data.startsWith("T")) {
      tiltAngle = data.substring(1).toInt(); // Get tilt angle
      tiltServo.write(tiltAngle);            // Move tilt servo
    }
  }
}
