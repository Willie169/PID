// Arduino Pins
#define L293D_LEFT1 12
#define L293D_LEFT2 11
#define L293D_RIGHT1 10
#define L293D_RIGHT2 9
#define TRIG_LEFT 14
#define TRIG_RIGHT 13
#define ECHO_LEFT 8
#define ECHO_RIGHT 7
// Multipliers
#define DISTANCE_OVER_DURATION 58.2
#define TARGET_DISTANCE 10
#define DISTANCE_BETWEEN_ULTRASONIC 5
#define DIFFERENTIAL_SPEED_MULTIPLIER 0.8
#define LEFT_POSITIVE_SPEED_MULTIPLIER 1
#define LEFT_NEGATIVE_SPEED_MULTIPLIER 1.2
#define RIGHT_POSITIVE_SPEED_MULTIPLIER 1
#define RIGHT_NEGATIVE_SPEED_MULTIPLIER 1.2
// debug or not
#define debug 1
// Adjust the above parameters
// The below code does not need to be changed

#include "PID.hpp"

PID avgPID(461.9, 517.9, 0.00265, 0.472, 1, 65.37, 1.7, 125, 0.15, 140, 0.2436);
PID difPID(461.9, 517.9, 0.00265, 0.472, 1, 65.37, 1.7, 125, 0.15, 140, 0.2436);
double avgV;
double difV;
String* ptr = new String();

inline double leftIn() {
    digitalWrite(TRIG_LEFT, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_LEFT, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_LEFT, LOW);
    return pulseIn(ECHO_LEFT, HIGH) * DISTANCE_OVER_DURATION;
}

inline double rightIn() {
    digitalWrite(TRIG_RIGHT, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_RIGHT, HIGH);
    delayMicroseconds(10);
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
    pinMode(L293D_LEFT1, OUTPUT);
    pinMode(L293D_LEFT2, OUTPUT);
    pinMode(L293D_RIGHT1, OUTPUT);
    pinMode(L293D_RIGHT2, OUTPUT);
    avgV = 0;
    difV = 0;
}

void loop() {
    double left = leftIn();
    double right = rightIn();
    double avg = left + right;
    double dif = atan2(left - right, DISTANCE_BETWEEN_ULTRASONIC);

    if (debug) avgV += avgPID.update((avg - TARGET_DISTANCE), millis(), ptr);
    else avgV += avgPID.update((avg - TARGET_DISTANCE), millis());
    if (debug) difV += difPID.update((dif - TARGET_DISTANCE), millis(), ptr) * DIFFERENTIAL_SPEED_MULTIPLIER;
    else difV += difPID.update((dif - TARGET_DISTANCE), millis()) * DIFFERENTIAL_SPEED_MULTIPLIER;

    double leftV = avgV + difV;
    double rightV = avgV - difV;

    leftOut(CLAMP(leftV, 255, -255));
    rightOut(CLAMP(rightV, 255, -255));

    delayMicroseconds(10000);
    stop();
}

inline void leftOut(double leftV) {
    if (debug) Serial.println("Left Velocity: " + String(leftV));
    leftV *= (leftV < 0)?LEFT_NEGATIVE_SPEED_MULTIPLIER:LEFT_POSITIVE_SPEED_MULTIPLIER;
    if (debug) Serial.println("Left Output: " + String(leftV));
    if (leftV > 0) {
        analogWrite(L293D_LEFT1, 0);
        analogWrite(L293D_LEFT2, leftV);
    } else {
        analogWrite(L293D_LEFT1, -leftV);
        analogWrite(L293D_LEFT2, 0);
    }
}

inline void rightOut(double rightV) {
    if (debug) Serial.println("Right Velocity: " + String(rightV));
    rightV *= (rightV < 0)?RIGHT_NEGATIVE_SPEED_MULTIPLIER:RIGHT_POSITIVE_SPEED_MULTIPLIER;
    if (debug) Serial.println("Right Output: " + String(rightV));
    if (rightV > 0) {
        analogWrite(L293D_RIGHT1, 0);
        analogWrite(L293D_RIGHT2, rightV);
    } else {
        analogWrite(L293D_RIGHT1, -rightV);
        analogWrite(L293D_RIGHT2, 0);
    }
}

inline void stop() {
    analogWrite(L293D_LEFT1, 0);
    analogWrite(L293D_LEFT2, 0);
    analogWrite(L293D_RIGHT1, 0);
    analogWrite(L293D_RIGHT2, 0);
}