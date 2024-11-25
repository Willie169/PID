#include "PID.hpp"
#define TARGET_DISTANCE 10
#define DISTANCE_BETWEEN_ULTRASONIC 5
#define DIFFERENTIAL_SPEED_MULTIPLIER 0.8
#define POSITIVE_SPEED_MULTIPLIER 1
#define NEGATIVE_SPEED_MULTIPLIER 1.2

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
    
    avgV += avgPID.update((avg - TARGET_DISTANCE), millis());
    difV += difPID.update((dif - TARGET_DISTANCE), millis()) * DIFFERENTIAL_SPEED_MULTIPLIER;
    
    double leftV = avgV + difV;
    leftV *= (leftV>0) ? POSITIVE_SPEED_MULTIPLIER : NEGATIVE_SPEED_MULTIPLIER;
    double rightV = avgV - difV;
    rightV *= (rightV>0) ? POSITIVE_SPEED_MULTIPLIER : NEGATIVE_SPEED_MULTIPLIER;
    leftOut();
    rightOut();
}

inline void leftOut(double output) {
    Serial.println("Left: " + String(output));
}

inline void rightOut(double output) {
    Serial.println("Right: " + String(output));
}
