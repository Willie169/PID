#ifndef OPTIMIZATION_HPP
#define OPTIMIZATION_HPP

#include <vector>
#include <iostream>
#include <thread>
#include <mutex>
#include <queue>
#include <atomic>
#include <chrono>
#include <iomanip>
#include <condition_variable>
#include <cmath>
#include "testCar.hpp"

using namespace std;

struct ParameterRange {
    double start;
    double end;
    double step;
};

mutex resultsMutex;
mutex queueMutex;
atomic<int> totalCombinations(0);
atomic<int> processedCombinations(0);
queue<vector<double>> parameterQueue;
condition_variable cv;

void displayProgress() {
    while (true) {
        {
            lock_guard<mutex> lock(queueMutex);
            if (processedCombinations >= totalCombinations) {
                break;
            }
            cout << "\rProgress: " << processedCombinations << "/" << totalCombinations 
                 << " (" << fixed << setprecision(2) 
                 << (static_cast<double>(processedCombinations) / totalCombinations * 100) 
                 << "%)          " << flush;
        }
        this_thread::sleep_for(chrono::seconds(1));
    }
    cout << "\rProgress: 100% Complete!                   " << endl;
}

void workerFunction(vector<pidTest>& results, atomic<bool>& running) {
    while (running) {
        vector<double> params;
        
        {
            lock_guard<mutex> lock(queueMutex);
            if (parameterQueue.empty()) {
                continue;
            }
            params = parameterQueue.front();
            parameterQueue.pop();
        }

        vector<double> tmp = test(
            params[0], params[1], params[2], params[3], 
            params[4], params[5], params[6], params[7], 
            params[8], params[9], params[10]
        );
        
        double result = sum_last_squared(tmp, 0.5);

        {
            lock_guard<mutex> lock(resultsMutex);
            results.emplace_back(pidTest{
                params[0], params[1], params[2], params[3],
                params[4], params[5], params[6], params[7],
                params[8], static_cast<unsigned long>(params[9]),
                params[10], result
            });
        }

        processedCombinations++;
        cv.notify_one();
    }
}

int optimize(vector<ParameterRange> ranges) {
    totalCombinations = 1;
    for (const auto& range : ranges) {
        totalCombinations *= static_cast<int>(ceil((range.end - range.start) / range.step)) + 1;
    }

    cout << "Total parameter combinations to test: " << totalCombinations << endl;

    for (double maxIntTm = ranges[0].start; maxIntTm <= ranges[0].end; maxIntTm += ranges[0].step) {
        for (double maxAmp = ranges[1].start; maxAmp <= ranges[1].end; maxAmp += ranges[1].step) {
            for (double minKp = ranges[2].start; minKp <= ranges[2].end; minKp += ranges[2].step) {
                for (double maxKp = max(minKp + ranges[3].step, ranges[3].start); 
                     maxKp <= ranges[3].end; maxKp += ranges[3].step) {
                    for (double rTiM = ranges[4].start; rTiM <= ranges[4].end; rTiM += ranges[4].step) {
                        for (double TdM = ranges[5].start; TdM <= ranges[5].end; TdM += ranges[5].step) {
                            for (double TddM = ranges[6].start; TddM <= ranges[6].end; TddM += ranges[6].step) {
                                for (double eDPm = ranges[7].start; eDPm <= ranges[7].end; eDPm += ranges[7].step) {
                                    for (double eDPa = ranges[8].start; eDPa <= ranges[8].end; eDPa += ranges[8].step) {
                                        for (double session = ranges[9].start; session <= ranges[9].end; session += ranges[9].step) {
                                            for (double Kp = max(minKp, ranges[10].start); 
                                                 Kp <= min(maxKp, ranges[10].end); Kp += ranges[10].step) {
                                                parameterQueue.push({
                                                    maxIntTm, maxAmp, minKp, maxKp, rTiM,
                                                    TdM, TddM, eDPm, eDPa, session, Kp
                                                });
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

    vector<pidTest> results;
    vector<thread> threads;
    atomic<bool> running(true);

    thread progressThread(displayProgress);
    unsigned int numThreads = thread::hardware_concurrency();
    cout << "Using " << numThreads << " threads" << endl;
    
    for (unsigned int i = 0; i < numThreads; ++i) {
        threads.emplace_back(workerFunction, ref(results), ref(running));
    }
    
    for (auto& thread : threads) {
        thread.join();
    }

    running = false;
    cv.notify_all();

    progressThread.join();
    
    if (!write_results(results, "results.csv")) {
        cerr << "Error writing results to file." << endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

#endif