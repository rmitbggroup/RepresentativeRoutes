#ifndef PERFORMANCETIMER_H_
#define PERFORMANCETIMER_H_

#include <chrono>
using namespace std::chrono;

struct PerformanceTimer
{
	microseconds sum;
	decltype(high_resolution_clock::now()) start;

	PerformanceTimer()
	{
		sum = microseconds(0);
	}
	void pause()
	{
		auto t = high_resolution_clock::now();
		sum += duration_cast<microseconds>(t - start);
	}
	void resume()
	{
		start = high_resolution_clock::now();
	}
	void clear()
	{
		sum = microseconds::zero();
	}
};


extern PerformanceTimer read_time, segment_time, connect_time, dist_time, olap_time;

#endif /* PERFORMANCETIMER_H_ */
