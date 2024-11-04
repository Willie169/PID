#include "optimization.hpp"

int main() {
    vector<ParameterRange> ranges = {
        {100, 1000, 100},    // maxIntTm
        {10, 100, 10},     // maxAmp
        {0, 1, 0.1},       // minKp
        {0.1, 1, 0.1},      // maxKp
        {1, 10, 1},       // rTiM
        {1, 10, 1},       // TdM
        {1, 10, 1},        // TddM
        {100, 1000, 100}, // eDPm
        {0, 0.1, 0.01},    // eDPa
        {0, 150, 10},       // session
        {0, 0.5, 0.01}      // Kp
    };
    
    return optimize(ranges);
}
