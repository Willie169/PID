#ifndef OPTIMIZATION_HPP
#define OPTIMIZATION_HPP

#include "testCar.hpp"
using namespace std;

int optimize(vector<ParameterRange> ranges)
{
    vector<pidTest> results;
    
	for (double maxIntTm = ranges[0].start; maxIntTm <= ranges[0].end; maxIntTm += ranges[0].step)
	{
		for (double maxAmp = ranges[1].start; maxAmp <= ranges[1].end; maxAmp += ranges[1].step)
		{
			for (double minKp = ranges[2].start; minKp <= ranges[2].end; minKp += ranges[2].step)
			{
				for (double maxKp = max(minKp + ranges[3].step, ranges[3].start);
					 maxKp <= ranges[3].end; maxKp += ranges[3].step)
				{
					for (double rTiM = ranges[4].start; rTiM <= ranges[4].end; rTiM += ranges[4].step)
					{
						for (double TdM = ranges[5].start; TdM <= ranges[5].end; TdM += ranges[5].step)
						{
							for (double TddM = ranges[6].start; TddM <= ranges[6].end; TddM += ranges[6].step)
							{
								for (double eDPm = ranges[7].start; eDPm <= ranges[7].end; eDPm += ranges[7].step)
								{
									for (double eDPa = ranges[8].start; eDPa <= ranges[8].end; eDPa += ranges[8].step)
									{
										for (unsigned long session = ranges[9].start; session <= ranges[9].end; session += ranges[9].step)
										{
											for (double Kp = max(minKp, ranges[10].start);
												 Kp <= min(maxKp, ranges[10].end); Kp += ranges[10].step)
											{
												vector<double> tmp = test(maxIntTm, maxAmp, minKp, maxKp, rTiM, TdM, TddM, eDPm, eDPa, session, Kp);
												double result = sum_last_squared(tmp, 0.5);
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

	if (!write_results(results, "results.csv"))
	{
		cerr << "Error writing results to file." << endl;
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}

#endif