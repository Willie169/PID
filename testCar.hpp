#ifndef TESTCAR_HPP
#define TESTCAR_HPP

#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <numeric>
#include <fstream>
#include <algorithm>
#include "PID.hpp"
using namespace std;

class Car
{
  public:
	Car(double initialSpeed, double initialPosition)
		: speed(initialSpeed), position(initialPosition) {}

	void updatePosition(double timeInterval)
	{
		position += speed * timeInterval;
	}

	void changeSpeed(double newSpeed)
	{
		speed = newSpeed;
	}

	double getPosition() const
	{
		return position;
	}

	double getSpeed() const
	{
		return speed;
	}

	double speed;
	double position;
};

void pidDebug(string message)
{
	cout << message;
}

double sum_last_squared(const std::vector<double> &v, double prop)
{
	if (v.empty())
		return 0.0;

	size_t n = v.size();
	size_t count = static_cast<size_t>(std::ceil(n * prop));
	size_t start_index = n - count;

	double sum_of_squares = std::accumulate(v.begin() + start_index, v.end(), 0.0,
											[](double sum, double value) {
												return sum + value * value;
											});
	return sum_of_squares;
}

vector<double> velocities(int steps)
{
	vector<double> velocities(steps, 0.0);
	double acceleration = 1;
	double error = 0.2;

	int phase_length = steps / 6;
	double velocity = 0.0;

	for (int i = 0; i < phase_length; ++i)
	{
		velocity += acceleration + ((i % 3 == 0) ? error : ((i % 3 == 1) ? (-error) : 0));
		velocities[i] = velocity;
	}

	for (int i = phase_length; i < 2 * phase_length; ++i)
	{
		velocity += ((i % 3 == 0) ? error : ((i % 3 == 1) ? (-error) : 0));
		velocities[i] = velocity;
	}

	for (int i = 2 * phase_length; i < 4 * phase_length; ++i)
	{
		velocity -= acceleration + ((i % 3 == 0) ? error : ((i % 3 == 1) ? (-error) : 0));
		velocities[i] = velocity;
	}

	for (int i = 4 * phase_length; i < 5 * phase_length; ++i)
	{
		velocity += acceleration + ((i % 3 == 0) ? error : ((i % 3 == 1) ? (-error) : 0));
		velocities[i] = velocity;
	}

	for (int i = 5 * phase_length; i < steps; ++i)
	{
		if (ABS(velocity) >= (error * (steps - i)))
		{
			velocity += COPYSIGN(velocity, error);
		}
		else
		{
			velocity += ((i % 3 == 0) ? error : ((i % 3 == 1) ? (-error) : 0));
		}
		velocities[i] = velocity;
	}

	return velocities;
}

vector<double> test(double maxIntTm, double maxAmp, double minKp, double maxKp, double rTiM, double TdM, double TddM, double eDPm, double eDPa, unsigned long session, double Kp, double preE = 0, unsigned long preT = 0, double timeInterval = 10, int steps = 1000, bool debug = true)
{
	double leaderInitialSpeed = 0;
	double followerInitialSpeed = 0;
	double initialDistance = 0;

	vector<double> vec;
	vector<double> vol = velocities(steps);

	Car leader(leaderInitialSpeed, initialDistance);
	Car follower(followerInitialSpeed, 0);

	PID pid = PID(maxIntTm, maxAmp, minKp, maxKp, rTiM, TdM, TddM, eDPm, eDPa, session, Kp, preE, preT);

	cout << fixed << setprecision(4);

	for (int i = 0; i < steps; i++)
	{
		leader.changeSpeed(leader.getSpeed() + vol[i]);
		leader.updatePosition(timeInterval);
		follower.updatePosition(timeInterval);

		double distanceToLeader = leader.getPosition() - follower.getPosition();
		vec.push_back(distanceToLeader);
		double fS;
		if (debug) {
            cout << "TimeStep: " << i + 1 << ", distanceToLeader: " << distanceToLeader << ", FollowerSpeed: " << follower.getSpeed() << ", LeaderSpeed: " << leader.getSpeed() << "\n";
		
		    string* ptr = new string;
		    fS = pid.update(distanceToLeader, i * timeInterval, ptr);
		} else fS = pid.update(distanceToLeader, i * timeInterval);
		follower.changeSpeed(follower.getSpeed() + MAX(-511, MIN(fS, 511)));
		if (debug) {
            cout << *ptr;
		    delete ptr;
		}
	}

	return vec;
}

struct pidTest
{
	double maxIntTm, maxAmp, minKp, maxKp, rTiM, TdM, TddM, eDPm, eDPa;
	unsigned long session;
	double Kp, result;
};

bool write_results(const vector<pidTest> &data, const string &filename)
{
	auto sorted_data = data;
	sort(sorted_data.begin(), sorted_data.end(), [](const pidTest &a, const pidTest &b) {
		return a.result < b.result;
	});

	ofstream outfile(filename);
	if (!outfile)
	{
		cerr << "Error opening file for writing." << endl;
		return 0;
	}

	outfile << "maxIntTm,maxAmp,minKp,maxKp,rTiM,TdM,TddM,eDPm,eDPa,session,Kp,result\n";

	for (size_t i = 0; i < sorted_data.size(); ++i)
	{
		const pidTest &pt = sorted_data[i];
		outfile << pt.maxIntTm << "," << pt.maxAmp << "," << pt.minKp << "," << pt.maxKp << ","
				<< pt.rTiM << "," << pt.TdM << "," << pt.TddM << "," << pt.eDPm << "," << pt.eDPa << ","
				<< pt.session << "," << pt.Kp << "," << pt.result << "\n";
	}

	outfile.close();
	cout << "Data written to " << filename << endl;

	return 1;
}

struct ParameterRange
{
	double start;
	double end;
	double step;
};

#endif