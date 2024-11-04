#include <vector>
#include <iostream>
#include <fstream>
#include <algorithm>
#include "testCar.hpp"
using namespace std;

struct pidTest {
    double maxIntTm, maxAmp, minKp, maxKp, rTiM, TdM, TddM, eDPm, eDPa;
    unsigned long session;
    double Kp, result;
};

void write_results(const std::vector<pidTest>& data, const std::string& filename) {
    auto sorted_data = data;
    std::sort(sorted_data.begin(), sorted_data.end(), [](const pidTest& a, const pidTest& b) {
        return a.result < b.result;
    });

    std::ofstream outfile(filename);
    if (!outfile) {
        std::cerr << "Error opening file for writing." << std::endl;
        return;
    }

    outfile << "maxIntTm,maxAmp,minKp,maxKp,rTiM,TdM,TddM,eDPm,eDPa,session,Kp,result\n";

    for (size_t i = 0; i < sorted_data.size(); ++i) {
        const pidTest& pt = sorted_data[i];
        outfile << pt.maxIntTm << "," << pt.maxAmp << "," << pt.minKp << "," << pt.maxKp << ","
                << pt.rTiM << "," << pt.TdM << "," << pt.TddM << "," << pt.eDPm << "," << pt.eDPa << ","
                << pt.session << "," << pt.Kp << "," << pt.result << "\n";
    }

    outfile.close();
    std::cout << "Data written to " << filename << std::endl;
}

int main() {
    vector<pidTest> results;

    for (double maxIntTm = 0; maxIntTm <= 100000; maxIntTm+=1) {
        for (double maxAmp = 0; maxAmp <= 100000; maxAmp += 1) {
            for (double minKp = 0; minKp <= 1; minKp += 0.001) {
                for (double maxKp = minKp; maxKp <= 1; maxKp += 0.001) {
                    for (double rTiM = 0; rTiM <= 1000; rTiM += 1) {
                        for (double TdM = 0; TdM < 1000; TdM += 1) {
                            for (double TddM = 0; TddM < 10; TddM += 0.1) {
                                for (double eDPm = 0; eDPm <= 10000; eDPm++) {
                                    for (double eDPa = 0; eDPa <= 0.5; eDPa += 0.001) {
                                        for (unsigned long session = 0; session < 5000; session += 100) {
                                            for (double Kp = minKp; Kp <= maxKp; Kp += 0.001) {
                                                vector<double> tmp = test(maxIntTm, maxAmp, minKp, maxKp, rTiM, TdM, TddM, eDPm, eDPa, session, Kp);
                                                double result = average_last(tmp, 0.5);
                                                results.push_back({maxIntTm, maxAmp, minKp, maxKp, rTiM, TdM, TddM, eDPm, eDPa, session, Kp, result});
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    write_results(results, "results.csv");

    return 0;
}