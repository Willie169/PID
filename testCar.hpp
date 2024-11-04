#ifndef TESTCAR_HPP
#define TESTCAR_HPP

#include <iostream>
#include <random>
#include <cmath>
#include <string>
#include <iomanip>
#include <vector>
#include <numeric>
#include "PID.hpp"
using namespace std;

class Car {
public:
    Car(double initialSpeed, double initialPosition)
        : speed(initialSpeed), position(initialPosition) {}

    void updatePosition(double timeInterval) {
        position += speed * timeInterval;
    }

    void changeSpeed(double newSpeed) {
        speed = newSpeed;
    }

    double getPosition() const {
        return position;
    }

    double getSpeed() const {
        return speed;
    }

    double speed;
    double position;
};

void pidDebug(string message) {
    cout << message;
}

vector<double> test(double maxIntTm, double maxAmp, double minKp, double maxKp, double rTiM, double TdM, double TddM, double eDPm, double eDPa, unsigned long session, double Kp, double preE=0, unsigned long preT=0) {
    vector<double> vec;
    
    random_device rd;
    mt19937 gen(rd());
    normal_distribution<> dist(0, 127.5);
    double random_num;
    
    double leaderInitialSpeed = 0;
    double followerInitialSpeed = 0;
    double initialDistance = 0;
    double timeInterval = 100;

    Car leader(leaderInitialSpeed, initialDistance);
    Car follower(followerInitialSpeed, 0);

    int steps = 10000;

    PID pid = PID(maxIntTm, maxAmp, minKp, maxKp, rTiM, TdM, TddM, eDPm, eDPa, session, Kp, preE, preT);

    cout << fixed << setprecision(4);

    for (int i = 0; i < steps; ++i) {
        do {
            random_num = dist(gen);
        } while (random_num < -255.0 || random_num > 255.0);
        
        leader.changeSpeed(leader.getSpeed() + random_num);
        leader.updatePosition(timeInterval);
        follower.updatePosition(timeInterval);
        
        double distanceToLeader = leader.getPosition() - follower.getPosition();
        vec.push_back(distanceToLeader);
//        cout << "TimeStep: " << i + 1 << ", distanceToLeader: " << distanceToLeader << ", FollowerSpeed: " << follower.getSpeed() << ", LeaderSpeed: " << leader.getSpeed() << "\n";
        
//        string* ptr = new string;
//        double fS = pid.update(distanceToLeader, i * timeInterval, ptr);
        double fS = pid.update(distanceToLeader, i * timeInterval);
        follower.changeSpeed(follower.getSpeed()+MAX(-511,MIN(fS,511)));
//        cout << *ptr;
//        delete ptr;
    }
    
    return vec;
}

double average_last(const vector<double>& v, double prop) {
    if (v.empty()) return 0.0;
    
    size_t n = v.size();
    size_t count = static_cast<size_t>(std::ceil(n * prop));
    size_t start_index = n - count;

    double sum = std::accumulate(v.begin() + start_index, v.end(), 0.0);
    return sum / count;
}

#endif