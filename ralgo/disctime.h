#ifndef RALGO_DISCRETE_TIME_H
#define RALGO_DISCRETE_TIME_H

#include <stdint.h>

namespace ralgo 
{
	int64_t discrete_time();
	float discrete_time_frequency();

	template<class Time> Time current_time() 
	{
		return (Time)discrete_time() / discrete_time_frequency();
	}
}

#endif