#include "PID.hpp"
#define TARGET 10
#define MULTIPLIER 0.5

PID pid(461.9, 517.9, 0.00265, 0.472, 1, 65.37, 1.7, 125, 0.15, 140, 0.2436);
double out;

double input() {
    return 0;
}

void setup() {
    out = 0;
}

void loop() {
    out += pid.update((input() - TARGET), millis()) * MULTIPLIER;
    output(out);
}

inline void output(double output) {
    Serial.println(output);
}