#include <iostream>
#include <ctime>
#include <iomanip>
#include <string>
#include "PID.hpp"

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

void pidDebug(std::string message) {
    std:: cout << message;
}

int main() {
    srand(static_cast<unsigned int>(time(0)));

    double leaderInitialSpeed = 0;
    double followerInitialSpeed = 0;
    double initialDistance = 0;
    double timeInterval = 10;

    Car leader(leaderInitialSpeed, initialDistance);
    Car follower(followerInitialSpeed, 0);

    int steps = 1000;

    PID pid = PID();

    std::cout << std::fixed << std::setprecision(4);

    for (int i = 0; i < steps; ++i) {
        leader.changeSpeed(leader.getSpeed() + static_cast<double>(rand() % 401 - 200) / 100);
        leader.updatePosition(timeInterval);
        follower.updatePosition(timeInterval);
        
        double distanceToLeader = leader.getPosition() - follower.getPosition();
        std::cout << "TimeStep: " << i + 1 << ", distanceToLeader: " << distanceToLeader << ", FollowerSpeed: " << follower.getSpeed() << ", LeaderSpeed: " << leader.getSpeed() << "\n";
        
        std::string* ptr = new std::string;
        double fS = pid.update(distanceToLeader, i * timeInterval, ptr);
        follower.changeSpeed(follower.getSpeed()+fS);
        std::cout << *ptr;
        delete ptr;
    }

    return 0;
}