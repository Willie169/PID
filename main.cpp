#include "optimization.hpp" // or #include "optimization_multithread.hpp"

int main() {
    // Note that steps can't be zero
    vector<ParameterRange> ranges = {
        {500, 500, 1},    // maxIntTm
        {50, 50, 1},     // maxAmp
        {0.01, 0.01, 1},       // minKp
        {1, 1, 1},      // maxKp
        {10, 10, 1},       // rTiM
        {10, 10, 1},       // TdM
        {1, 1, 1},        // TddM
        {300, 300, 1}, // eDPm
        {0.01, 0.01, 1},    // eDPa
        {0, 0, 1},       // session
        {0.01, 0.5, 0.01}      // Kp
    };
    
    return optimize(ranges);
}