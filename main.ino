// Arduino Pins
#define L293D_LEFT_IN1 12
#define L293D_LEFT_IN2 11
#define L293D_RIGHT_IN1 10
#define L293D_RIGHT_IN2 9
#define TRIG_LEFT 6
#define TRIG_RIGHT 5
#define ECHO_LEFT 8
#define ECHO_RIGHT 7
// Multipliers
#define HALF_SOUND_SPEED 0.1715
#define TARGET_DISTANCE 10
#define DISTANCE_BETWEEN_ULTRASONIC_SENSORS 5
#define ANGULAR_SPEED_MULTIPLIER 0.8
#define LEFT_POSITIVE_SPEED_MULTIPLIER 1
#define LEFT_NEGATIVE_SPEED_MULTIPLIER 1.2
#define RIGHT_POSITIVE_SPEED_MULTIPLIER 1
#define RIGHT_NEGATIVE_SPEED_MULTIPLIER 1.2
// 1 for debug, 0 for not
#define DEBUG 1
// Adjust the above parameters
// The below code does not need to be changed

#include "PID.hpp"

PID avgPID(461.9, 517.9, 0.00265, 0.472, 1, 65.37, 1.7, 125, 0.15, 140, 0.2436);
PID angPID(461.9, 517.9, 0.00265, 0.472, 1, 65.37, 1.7, 125, 0.15, 140, 0.2436);
double avgV;
double angV;
#if DEBUG
    String* ptr = new String();
#endif

inline double leftIn() {
    digitalWrite(TRIG_LEFT, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_LEFT, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_LEFT, LOW);
    return pulseIn(ECHO_LEFT, HIGH) * HALF_SOUND_SPEED;
}

inline double rightIn() {
    digitalWrite(TRIG_RIGHT, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_RIGHT, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_RIGHT, LOW);
    return pulseIn(ECHO_RIGHT, HIGH) * HALF_SOUND_SPEED;
}

void setup()
{ 
    Serial.begin (9600);
    pinMode(TRIG_LEFT, OUTPUT);
    pinMode(TRIG_RIGHT, OUTPUT);
    pinMode(ECHO_LEFT, INPUT);
    pinMode(ECHO_RIGHT, INPUT);
    pinMode(L293D_LEFT_IN1, OUTPUT);
    pinMode(L293D_LEFT_IN2, OUTPUT);
    pinMode(L293D_RIGHT_IN1, OUTPUT);
    pinMode(L293D_RIGHT_IN2, OUTPUT);
    avgV = 0;
    angV = 0;
}

void loop() {
    double left = leftIn();
    double right = rightIn();
    double avg = left + right;
    double ang = atan2(left - right, DISTANCE_BETWEEN_ULTRASONIC_SENSORS);

    #if DEBUG
        ptr->clear();
        avgV += avgPID.update((avg - TARGET_DISTANCE), millis(), ptr);
        Serial.println("Average Speed PID Update:");
        Serial.println(*ptr);
        ptr->clear();
        angV += angPID.update((ang - TARGET_DISTANCE), millis(), ptr) * ANGULAR_SPEED_MULTIPLIER;
        Serial.println("Angular Speed Update:");
        Serial.println(*ptr);
        ptr->clear();
    #else
        avgV += avgPID.update((avg - TARGET_DISTANCE), millis());
        angV += angPID.update((ang - TARGET_DISTANCE), millis()) * ANGULAR_SPEED_MULTIPLIER;
    #endif

    double leftV = avgV + angV;
    double rightV = avgV - angV;

    leftOut(CLAMP(leftV, 255, -255));
    rightOut(CLAMP(rightV, 255, -255));

    delayMicroseconds(10000);
}

inline void leftOut(double leftV) {
    #if DEBUG
        Serial.println("Left Velocity: " + String(leftV));
    #endif
    leftV *= (leftV < 0)?LEFT_NEGATIVE_SPEED_MULTIPLIER:LEFT_POSITIVE_SPEED_MULTIPLIER;
    #if DEBUG
        Serial.println("Left Output: " + String(leftV));
    #endif
    if (leftV > 0) {
        analogWrite(L293D_LEFT_IN1, 0);
        analogWrite(L293D_LEFT_IN2, leftV);
    } else {
        analogWrite(L293D_LEFT_IN1, -leftV);
        analogWrite(L293D_LEFT_IN2, 0);
    }
}

inline void rightOut(double rightV) {
    #if DEBUG
        Serial.println("Right Velocity: " + String(rightV));
    #endif
    rightV *= (rightV < 0)?RIGHT_NEGATIVE_SPEED_MULTIPLIER:RIGHT_POSITIVE_SPEED_MULTIPLIER;
    #if DEBUG
        Serial.println("Right Output: " + String(rightV));
    #endif
    if (rightV > 0) {
        analogWrite(L293D_RIGHT_IN1, 0);
        analogWrite(L293D_RIGHT_IN2, rightV);
    } else {
        analogWrite(L293D_RIGHT_IN1, -rightV);
        analogWrite(L293D_RIGHT_IN2, 0);
    }
}
