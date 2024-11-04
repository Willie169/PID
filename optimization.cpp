#ifndef OPTIMIZATION_CPP
#define OPTIMIZATION_CPP

#include <vector>
#include <iostream>
#include "testCar.hpp"
using namespace std;

int main()
{
	vector<pidTest> results;

	for (double maxIntTm = 10; maxIntTm <= 1000; maxIntTm += 10)
	{
		for (double maxAmp = 0; maxAmp <= 1000; maxAmp += 10)
		{
			for (double minKp = 0; minKp <= 1; minKp += 0.01)
			{
				for (double maxKp = MAX(0.01, minKp); maxKp <= 1; maxKp += 0.01)
				{
					for (double rTiM = 0; rTiM <= 100; rTiM += 1)
					{
						for (double TdM = 0; TdM < 100; TdM += 1)
						{
							for (double TddM = 0; TddM < 10; TddM += 0.1)
							{
								for (double eDPm = 0; eDPm <= 10000; eDPm += 10)
								{
									for (double eDPa = 0; eDPa <= 0.5; eDPa += 0.005)
									{
										for (unsigned long session = 0; session < 500; session += 10)
										{
											for (double Kp = minKp; Kp <= maxKp; Kp += 0.001)
											{
												vector<double> tmp = test(maxIntTm, maxAmp, minKp, maxKp, rTiM, TdM, TddM, eDPm, eDPa, session, Kp);
												double result = sum_last_squared(tmp, 0.5);
												results.push_back({maxIntTm, maxAmp, minKp, maxKp, rTiM, TdM, TddM, eDPm, eDPa, session, Kp, result});
												cout << result << "\n";
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

#endif