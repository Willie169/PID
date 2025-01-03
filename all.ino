// Arduino Pins
#define L293D_LEFT_EN 2
#define L293D_RIGHT_EN 3
#define L293D_LEFT_IN1 5
#define L293D_LEFT_IN2 6
#define L293D_RIGHT_IN1 7
#define L293D_RIGHT_IN2 8
#define TRIG_LEFT 4
#define TRIG_RIGHT 9
#define ECHO_LEFT 10
#define ECHO_RIGHT 11
// Constants
#define HALF_SOUND_SPEED 0.1715
#define TARGET_DISTANCE 15
#define DISTANCE_BETWEEN_ULTRASONIC_SENSORS 12
// Parameters to be tested
#define ANGULAR_SPEED_MULTIPLIER 0.8
#define LEFT_POSITIVE_SPEED_MULTIPLIER 1
#define LEFT_NEGATIVE_SPEED_MULTIPLIER 1.2
#define RIGHT_POSITIVE_SPEED_MULTIPLIER 1
#define RIGHT_NEGATIVE_SPEED_MULTIPLIER 1.2
// 1 for debug, 0 for not
#define DEBUG 1
// Adjust the above parameters
// The below code does not need to be changed

#ifdef ARDUINO
#include <Arduino.h>
#include <math.h>
#define to_string(a) String(a)
#else
#include <cmath>
#include <string>
#define String string
using namespace std;
#endif
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define ABS(a) ((a) < 0 ? -(a) : (a))
#define COPYSIGN(a, b) ((b) < 0 ? -ABS(a) : ABS(a))
#define CLAMP(a, b, c) (MAX(MIN((a), (b)), (c)))

struct Data {
    unsigned long dt;
    double e;
};

struct Node {
    Data data;
    Node* prev;
};

struct List {
    Node* head;
    Node* tail;
    unsigned int size;
};

class PID {
public:
    const double maxIntTm, maxAmp, minKp, maxKp, rTiM, TdM, TddM, eDPm, eDPa;
    const unsigned long session;
    double Kp, Ki, Kd, Kdd, preE, preDv, intg, preOut;
    unsigned long preT;
    List dtXs;
    
    PID(double maxIntTm, double maxAmp, double minKp, double maxKp, double rTiM, double TdM, double TddM, double eDPm, double eDPa, unsigned long session, double Kp, double preE=0, unsigned long preT=0)
        : maxIntTm(maxIntTm), maxAmp(maxAmp), minKp(minKp), maxKp(maxKp), rTiM(rTiM), TdM(TdM), TddM(TddM), eDPm(eDPm), eDPa(eDPa), session(session), Kp(Kp), Ki(0), Kd(0), Kdd(0), preE(preE), preDv(0), intg(0), preOut(0), preT(preT), dtXs({nullptr, nullptr, 0}) {}
    
    double update(double e, unsigned long timestamp, String* debug = nullptr) {
        unsigned long dt = timestamp - preT;
        preT = timestamp;
        
        if (dt == 0) {
            if (debug != nullptr) {
                *debug += "dt=0\n";
            }
            return preOut;
        }
        
        double amp, ap;

        if (session>0) {
	        unsigned long S = 0;
	        amp = 0;
	        if (dtXs.size > 1) {
	            Node* pos = dtXs.tail;
	            while (dtXs.size > 1 && pos != nullptr) {
	                S += pos->data.dt;
	                amp = MAX(ABS(pos->data.e), amp);
	                if (S > session) {
	                    dtXs.head = pos;
	                    Node* cur = pos->prev;
	                    pos->prev = nullptr;
	                    while (cur != nullptr) {
	                        Node* tmp = cur->prev;
	                        delete cur;
	                        cur = tmp;
	                        dtXs.size--;
	                    }
	                    break;
	                } else pos = pos->prev;
	            }
	        } else if (dtXs.size == 1) {
	            amp = ABS(dtXs.tail->data.e);
	        }        
            ap = exp(-amp / maxAmp);
        } else {
            amp=0;
            ap=1;
        }
           
        Ki = Kp * rTiM * ap;
        Kd = Kp * TdM * ap;
        Kdd = Kd * TddM;
        
        double dv = (e - preE) / dt;
        double dd = (dv - preDv) / dt;

        intg += e * dt;
        double intTm = Ki * intg;
        if (ABS(intTm) > maxIntTm) {
            intg = COPYSIGN(maxIntTm / Ki, intTm);
            intTm = COPYSIGN(maxIntTm, intTm);
        }

        double propTm = Kp * e;
        double dvTm = Kd * dv;
        double ddTm = Kdd * dd;
        double out = propTm + intTm + dvTm + ddTm;
        
        double eDP = e * dv;
        if (ABS(eDP) > eDPm) {
            Kp *= (eDP < 0) ? (1+eDPa) : (1-eDPa);
            Kp = MIN(MAX(Kp, minKp), maxKp);
        }
        
        if (debug != nullptr) {
            *debug += "update() called\ndt: " + to_string(dt) + ", e: " + to_string(e) + ", dv: " + to_string(dv) + ", dd: " + to_string(dd) +", amp: " + to_string(amp) +", ap: " + to_string(ap) + ",\nKp: " + to_string(Kp) + ", Ki: " + to_string(Ki) + ", Kd: " + to_string(Kd) + ", Kdd: " + to_string(Kdd) + ",\npropTm: " + to_string(propTm) + ", intTm: " + to_string(intTm) + ", dvTm: " + to_string(dvTm) + ", ddTm: " + to_string(ddTm) + ",\neDP: " + to_string(eDP) + ", out: " + to_string(out) + "\n";
        }
        
        preE = e;
        preDv = dv;
        preOut = out;
 
        if (session>0) {
            Node* nn = new Node{{dt, e}, dtXs.tail};
            dtXs.tail = nn;
            if (dtXs.size == 0) dtXs.head = nn;
            dtXs.size++;
       } 
        
        return out;
    }
};

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
        *ptr = "";
        avgV += avgPID.update((avg - TARGET_DISTANCE), millis(), ptr);
        Serial.println("Average Speed PID Update:");
        Serial.println(*ptr);
        *ptr = "";
        angV += angPID.update((ang - TARGET_DISTANCE), millis(), ptr) * ANGULAR_SPEED_MULTIPLIER;
        Serial.println("Angular Speed Update:");
        Serial.println(*ptr);
        *ptr = "";
    #else
        avgV += avgPID.update((avg - TARGET_DISTANCE), millis());
        angV += angPID.update((ang - TARGET_DISTANCE), millis()) * ANGULAR_SPEED_MULTIPLIER;
    #endif

    double leftV = avgV + angV;
    double rightV = avgV - angV;

    leftOut(leftV);
    rightOut(rightV);

    delayMicroseconds(10000);
}

inline void leftOut(double leftV) {
    #if DEBUG
        Serial.println("Left Original Velocity: " + String(leftV));
    #endif
    leftV *= (leftV < 0)?LEFT_NEGATIVE_SPEED_MULTIPLIER:LEFT_POSITIVE_SPEED_MULTIPLIER;
    leftV = CLAMP(leftV, 255, -255);
    #if DEBUG
        Serial.println("Left Outputted Velocity: " + String(leftV));
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
        Serial.println("Right Original Velocity: " + String(rightV));
    #endif
    rightV *= (rightV < 0)?RIGHT_NEGATIVE_SPEED_MULTIPLIER:RIGHT_POSITIVE_SPEED_MULTIPLIER;
    rightV = CLAMP(rightV, 255, -255);
    #if DEBUG
        Serial.println("Right Outputted Velocity: " + String(rightV));
    #endif
    if (rightV > 0) {
        analogWrite(L293D_RIGHT_IN1, 0);
        analogWrite(L293D_RIGHT_IN2, rightV);
    } else {
        analogWrite(L293D_RIGHT_IN1, -rightV);
        analogWrite(L293D_RIGHT_IN2, 0);
    }
}

