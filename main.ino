#include "PID.hpp"
#define TARGET_DISTANCE 10
#define DISTANCE_BETWEEN_ULTRASONIC 5
#define DIFFERENTIAL_SPEED_MULTIPLIER 0.8
#define LEFT_POSITIVE_SPEED_MULTIPLIER 1
#define LEFT_NEGATIVE_SPEED_MULTIPLIER 1.2
#define RIGHT_POSITIVE_SPEED_MULTIPLIER 1
#define RIGHT_NEGATIVE_SPEED_MULTIPLIER 1.2
#define DISTANCE_OVER_DURATION 58.2

PID avgPID(461.9, 517.9, 0.00265, 0.472, 1, 65.37, 1.7, 125, 0.15, 140, 0.2436);
PID difPID(461.9, 517.9, 0.00265, 0.472, 1, 65.37, 1.7, 125, 0.15, 140, 0.2436);
double avgV;
double difV;

inline double leftIn() {
    digitalWrite(TRIG_LEFT, HIGH);
    digitalWrite(TRIG_LEFT, LOW);
    return pulseIn(ECHO_LEFT, HIGH) * DISTANCE_OVER_DURATION;
}

inline double rightIn() {
    digitalWrite(TRIG_RIGHT, HIGH);
    digitalWrite(TRIG_RIGHT, LOW);
    return pulseIn(ECHO_RIGHT, HIGH) * DISTANCE_OVER_DURATION;
}

void setup()
{ 
    Serial.begin (9600);
    pinMode(TRIG_LEFT, OUTPUT);
    pinMode(TRIG_RIGHT, OUTPUT);
    pinMode(ECHO_LEFT, INPUT);
    pinMode(ECHO_RIGHT, INPUT);
    avgV = 0;
    difV = 0;
}

void loop() {
    double left = leftIn();
    double right = rightIn();
    double avg = left + right;
    double dif = atan2(left - right, DISTANCE_BETWEEN_ULTRASONIC);
    
    avgV += avgPID.update((avg - TARGET_DISTANCE), millis());
    difV += difPID.update((dif - TARGET_DISTANCE), millis()) * DIFFERENTIAL_SPEED_MULTIPLIER;

    leftOut(avgV + difV);
    rightOut(avgV - difV);
}

inline void leftOut(double leftV) {
    Serial.println("Left Velocity: " + String(leftV));
    leftV *= (leftV < 0)?LEFT_NEGATIVE_SPEED_MULTIPLIER:LEFT_POSITIVE_SPEED_MULTIPLIER
}

inline void rightOut(double rightV) {
    Serial.println("Right Velocity: " + String(rightV));
    rightV *= (rightV < 0)?RIGHT_NEGATIVE_SPEED_MULTIPLIER:RIGHT_POSITIVE_SPEED_MULTIPLIER
}