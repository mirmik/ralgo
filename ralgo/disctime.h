#ifndef RALGO_DISCRETE_TIME_H
#define RALGO_DISCRETE_TIME_H

#include <stdint.h>

#if __has_include(<asm/heimer_types.h>)
#include <asm/heimer_types.h>
#else
typedef int64_t disctime_t;

disctime_t discrete_time();
float discrete_time_frequency();
#endif

#ifdef __cplusplus

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

#endif