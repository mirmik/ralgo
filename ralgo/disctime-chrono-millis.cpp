#include <ralgo/planning/disctime.h>
#include <chrono>

#include <stdint.h>

int64_t ralgo::discrete_time()
{
	return std::chrono::duration_cast< std::chrono::milliseconds >(
	           std::chrono::system_clock::now().time_since_epoch()).count();
}

float ralgo::discrete_time_frequency()
{
	return 1000;
}
