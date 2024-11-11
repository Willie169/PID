#include "PID.hpp"
#define TARGET_DISTANCE 10
#define AVERAGE_VELOCITY_MULTIPLIER 0.5
#define DIFFERENTIAL_VELOCITY_MULTIPLIER 0.5
#define DISTANCE_BETWEEN_ULTRASONIC 5

PID avgPID(461.9, 517.9, 0.00265, 0.472, 1, 65.37, 1.7, 125, 0.15, 140, 0.2436);
PID difPID(461.9, 517.9, 0.00265, 0.472, 1, 65.37, 1.7, 125, 0.15, 140, 0.2436);
double avgV;
double difV;

inline double leftIn() {
    return 0;
}

inline double rightIn() {
    return 0;
}

void setup() {
    avgV = 0;
    difV = 0;
}

void loop() {
    double left = leftIn();
    double right = rightIn();
    double avg = left + right;
    double dif = atan2(left - right, DISTANCE_BETWEEN_ULTRASONIC);
    
    avgV += avgPID.update((avg - TARGET_DISTANCE), millis()) * AVERAGE_VELOCITY_MULTIPLIER;
    difV += difPID.update((dif - TARGET_DISTANCE), millis()) * DIFFERENTIAL_VELOCITY_MULTIPLIER;
    
    leftOut(avgV + difV);
    rightOut(avgV - difV);
}

inline void leftOut(double output) {
    Serial.println("Left: " + String(output));
}

inline void rightOut(double output) {
    Serial.println("Right: " + String(output));
}
