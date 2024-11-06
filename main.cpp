#include "optimization.hpp" // or #include "optimization_multithread.hpp"

int main() {
    vector<ParameterRange> ranges = {
        {500, 500, 0},    // maxIntTm
        {50, 50, 0},     // maxAmp
        {0.01, 0.01, 0},       // minKp
        {1, 1, 0},      // maxKp
        {10, 10, 0},       // rTiM
        {10, 10, 0},       // TdM
        {1, 1, 0},        // TddM
        {300, 300, 0}, // eDPm
        {0.01, 0.01, 0},    // eDPa
        {0, 0, 0},       // session
        {0.01, 0.5, 0.01}      // Kp
    };
    
    return optimize(ranges);
}