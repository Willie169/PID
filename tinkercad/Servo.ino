#include <Servo.h>

const int trigPin = 12; // Trig Pin
const int echoPin = 11; // Echo Pin
const int greenLedPin = 9; // Green LED pin
const int redLedPin = 10; // Red LED pin
long duration, cm; // Duration and distance in cm

Servo servo_2;

enum Direction { CLOCKWISE, COUNTERCLOCKWISE, NONE }; // Enum for servo direction
Direction lastDirection = NONE; // Variable to store the last direction

void setup() { 
  Serial.begin(9600); // Set baud rate
  pinMode(trigPin, OUTPUT); // Set trigPin as OUTPUT
  pinMode(echoPin, INPUT); // Set echoPin as INPUT
  pinMode(greenLedPin, OUTPUT); // Set green LED pin as OUTPUT
  pinMode(redLedPin, OUTPUT); // Set red LED pin as OUTPUT
  servo_2.attach(2, 500, 2500); // Attach the servo to pin 2
}

void loop() {
  // Measure distance
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH); // Get the duration of the echo
  cm = (duration / 2) / 29.1; // Convert duration to distance in cm
  
  Serial.print("Distance: ");
  Serial.print(cm);
  Serial.println(" cm");
  
  // Control the servo and LEDs based on the distance
  if (cm > 160) {
    if (lastDirection != CLOCKWISE) {
      servo_2.write(servo_2.read() + 180); // Move clockwise
      lastDirection = CLOCKWISE;
    }
    digitalWrite(greenLedPin, HIGH); // Turn on green LED
    digitalWrite(redLedPin, LOW); // Turn off red LED
    Serial.println("Moving clockwise, green LED on");
  } else if (cm < 150) {
    if (lastDirection != COUNTERCLOCKWISE) {
      servo_2.write(servo_2.read() - 180); // Move counterclockwise
      lastDirection = COUNTERCLOCKWISE;
    }
    digitalWrite(greenLedPin, LOW); // Turn off green LED
    digitalWrite(redLedPin, HIGH); // Turn on red LED
    Serial.println("Moving counterclockwise, red LED on");
  }

  delay(100); // Wait before the next measurement
}
