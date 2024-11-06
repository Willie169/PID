#include "optimization.hpp" // or #include "optimization_multithread.hpp"

int main() {
    // Note that steps can't be zero
    vector<ParameterRange> ranges = {
        {100, 1000, 100},    // maxIntTm
        {50, 500, 50},     // maxAmp
        {0.01, 0.01, 1},       // minKp
        {1, 1, 1},      // maxKp
        {10, 100, 10},       // rTiM
        {10, 100, 10},       // TdM
        {1, 1, 1},        // TddM
        {100, 1000, 10}, // eDPm
        {0.01, 0.05, 0.01},    // eDPa
        {0, 150, 10},       // session
        {0.01, 1, 0.01}      // Kp
    };
    
    return optimize(ranges);
}
