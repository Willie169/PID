/**
 * Living Technology Ultrasonic Motor Control System
 * 
 * This system uses an ultrasonic sensor to detect distances and control a servo motor
 * and two relay pins based on the measured distance. LEDs are used for visual indicators.
 * A relay is triggered to activate another system depending on whether the measured 
 * distance falls within the defined threshold.
 * 
 * Components used:
 * - Ultrasonic Sensor (HC-SR04)
 * - Servo motor
 * - Two relays for controlling external circuits
 * - LEDs (green for 'safe', red for 'warning')
 * 
 * The system operates by emitting an ultrasonic pulse and measuring the time it takes 
 * for the echo to return. The distance is then calculated and compared to a predefined 
 * threshold. If the distance is greater than or less than the threshold, the system 
 * performs specific actions (such as moving the servo motor or switching relays).
 */

#include <Servo.h>

// Pin definitions (binary values for Arduino pins)
uint8_t trigPin = 0b1100;   // Pin for the ultrasonic sensor trigger
uint8_t echoPin = 0b1011;   // Pin for the ultrasonic sensor echo
const int green_led = 0b1101;   // Green LED pin for 'safe' state
const int red_led = 0b1000;     // Red LED pin for 'warning' state
const int current = 0b1001;     // Pin for current control (high when active)
uint32_t duration, cm;          // Variables to hold pulse duration and calculated distance
const uint16_t dist = 100;      // Threshold distance in cm
const uint8_t buffer = 5;       // Buffer distance to avoid rapid switching near the threshold
Servo servo1;                   // Servo motor object
uint8_t state = 0;              // State variable to track the system status
const int relayPin1 = 0b10;     // Relay pin 1
const int relayPin2 = 0b11;     // Relay pin 2

/**
 * Sets up the system by initializing the serial communication, pin modes,
 * and attaching the servo motor to the specified pin.
 */
void setup() {
    Serial.begin(9600);            // Start serial communication for debugging
    pinMode(trigPin, OUTPUT);      // Set trigger pin as output
    pinMode(echoPin, INPUT);       // Set echo pin as input
    pinMode(current, OUTPUT);      // Set current control pin as output
    pinMode(relayPin1, OUTPUT);    // Set relay pin 1 as output
    pinMode(relayPin2, OUTPUT);    // Set relay pin 2 as output
    servo1.attach(7, 500, 2500);   // Attach servo to pin 7 with pulse width constraints
}

/**
 * Main loop that continuously measures distance, controls LEDs, servo, and relays
 * based on the distance to the nearest object.
 */
void loop() {

    // Reset trigger and LED pins
    digitalWrite(trigPin, LOW);
    digitalWrite(green_led, LOW);
    digitalWrite(red_led, LOW);
    digitalWrite(current, HIGH);   // Enable current control

    delayMicroseconds(5);          // Short delay to stabilize the sensor

    // Send trigger pulse to ultrasonic sensor
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);         // Send pulse for 10 microseconds
    digitalWrite(trigPin, LOW);

    // Measure the duration of the echo pulse
    duration = pulseIn(echoPin, HIGH);

    // Calculate distance in cm
    cm = duration * 0.01715;

    // Update the state based on the distance
    if (cm > dist + buffer) {
        state = 1;   // Object is far enough, set state to 'safe'
    } else if (cm < dist) {
        state = 0;   // Object is too close, set state to 'warning'
    }

    // Control the servo and LEDs based on the state
    if (state == 1) {
        digitalWrite(green_led, HIGH);  // Turn on green LED (safe state)
        servo1.write(180);              // Move servo to 180 degrees
    } else {
        digitalWrite(red_led, HIGH);    // Turn on red LED (warning state)
        servo1.write(0);                // Move servo to 0 degrees
    }

    delayMicroseconds(10);  // Short delay for stability

    // Control the relay based on the distance
    if (cm < dist) {
        digitalWrite(relayPin1, HIGH);  // Activate relay 1 when object is too close
        digitalWrite(relayPin2, LOW);
    } else if (cm > dist + buffer) {
        digitalWrite(relayPin1, LOW);
        digitalWrite(relayPin2, HIGH);  // Activate relay 2 when object is far
    } else {
        digitalWrite(relayPin1, LOW);
        digitalWrite(relayPin2, LOW);   // Deactivate both relays when within the buffer
    }

    // Print the measured distance to the Serial Monitor
    Serial.print("Distance: ");
    Serial.print(cm);
    Serial.println(" cm");

    delay(250);   // Delay for stability between loops
}