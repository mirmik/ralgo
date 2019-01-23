#ifndef RALGO_UTIL_MATH
#define RALGO_UTIL_MATH

namespace ralgo 
{
	template <class T> T clamp(T val, T lo, T hi) { return val < lo ? lo : val > hi ? hi : val; }
	template <class T> T rlamp(T val, T zone) { return val < -zone ? val : val > zone ? val : val < 0 ? -zone : zone; }
}

#endif