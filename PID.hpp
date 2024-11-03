#ifndef PID_HPP
#define PID_HPP

#ifdef ARDUINO
#include <Arduino.h>
#include <math.h>
#define to_string(a) String(a)
#define copysign(a, b) (((b) < 0) ? - abs(a) : abs(a))
#else
#include <cmath>
#include <string>
#include <algorithm>
#define String string
using namespace std;
#endif

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
    List outs;
    
    PID(double maxIntTm=100, double maxAmp=1000, double minKp=0.001, double maxKp=0.01, double rTiM=1, double TdM=1, double TddM=0.1, double eDPm=50, double eDPa=0.1, unsigned long session=50, double Kp=0.005, double preE=0, unsigned long preT=0)
        : maxIntTm(maxIntTm), maxAmp(maxAmp), minKp(minKp), maxKp(maxKp), rTiM(rTiM), TdM(TdM), TddM(TddM), eDPm(eDPm), eDPa(eDPa), session(session), Kp(Kp), Ki(0), Kd(0), Kdd(0), preE(preE), preDv(0), intg(0), preOut(0), preT(preT), outs({nullptr, nullptr, 0}) {}
    
    double update(double e, unsigned long timestamp, String* debug = nullptr) {
        unsigned long dt = timestamp - preT;
        preT = timestamp;
        
        if (dt == 0) {
            if (debug != nullptr) {
                *debug += "dt=0\n";
            }
            return preOut;
        }

        unsigned long S = 0;
        double amp = 0;
        if (outs.size > 1) {
            Node* pos = outs.tail;
            while (outs.size > 1 && pos != nullptr) {
                S += pos->data.dt;
                amp = max(abs(pos->data.e), amp);
                if (S > session) {
                    outs.head = pos;
                    Node* cur = pos->prev;
                    pos->prev = nullptr;
                    while (cur != nullptr) {
                        Node* tmp = cur->prev;
                        delete cur;
                        cur = tmp;
                        outs.size--;
                    }
                    break;
                } else pos = pos->prev;
            }
        } else if (outs.size == 1) {
            amp = abs(outs.tail->data.e);
        }
        
        double ap = exp(-amp / maxAmp);
        Ki = Kp * rTiM * ap;
        Kd = Kp * TdM * ap;
        Kdd = Kd * TddM;
        
        double dv = (e - preE) / dt;
        double dd = (dv - preDv) / dt;

        intg += e * dt;
        double intTm = Ki * intg;
        if (abs(intTm) > maxIntTm) {
            intg = copysign(maxIntTm / Ki, intTm);
            intTm = copysign(maxIntTm, intTm);
        }

        double propTm = Kp * e;
        double dvTm = Kd * dv;
        double ddTm = Kdd * dd;
        double out = propTm + intTm + dvTm + ddTm;
        
        double eDP = abs(e * dv);
        if (eDP > eDPm) {
            Kp *= (eDP < 0) ? (1+eDPa) : (1-eDPa);
            Kp = min(max(Kp, minKp), maxKp);
        }
        
        if (debug != nullptr) {
            *debug += "update() called\ndt: " + to_string(dt) + ", e: " + to_string(e) + ", dv: " + to_string(dv) + ", dd: " + to_string(dd) +", amp: " + to_string(amp) + ",\nKp: " + to_string(Kp) + ", Ki: " + to_string(Ki) + ", Kd: " + to_string(Kd) + ", Kdd: " + to_string(Kdd) + ",\npropTm: " + to_string(propTm) + ", intTm: " + to_string(intTm) + ", dvTm: " + to_string(dvTm) + ", ddTm: " + to_string(ddTm) + ",\neDP: " + to_string(eDP) + ", out: " + to_string(out) + "\n";
        }
        
        preE = e;
        preDv = dv;
        Node* nn = new Node{{dt, e}, outs.tail};
        outs.tail = nn;
        if (outs.size == 0) outs.head = nn;
        outs.size++;
        preOut = out;
        
        return out;
    }
};

#endif