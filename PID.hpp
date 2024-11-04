#ifndef PID_HPP
#define PID_HPP

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
#define ABS(a) (((a) < 0) ? -(a) : (a))
#define COPYSIGN(a, b) (((b) < 0) ? -ABS(a) : ABS(a))

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

        if (session>0) {
	        unsigned long S = 0;
	        double amp = 0;
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
            double ap = exp(-amp / maxAmp);
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

#endif