#include "optimization.hpp" // or #include "optimization_multithread.hpp"

int main() {
    // Note that steps can't be zero
    vector<ParameterRange> ranges = {
        {461.9, 461.9, 1},    // maxIntTm
        {517.9, 517.9, 1},     // maxAmp
        {0.00265, 0.00265, 1},       // minKp
        {0.472, 0.472, 1},      // maxKp
        {1, 1, 1},       // rTiM
        {65.37, 65.37, 1},       // TdM
        {1.7, 1.7, 1},        // TddM
        {125, 125, 1}, // eDPm
        {0.15, 0.15, 1},    // eDPa
        {140, 140, 10},       // session
        {0.2436, 0.2436, 1}      // Kp
    };
    
    return optimize(ranges);
}
