#ifndef RALGO_UTIL_MATH
#define RALGO_UTIL_MATH

namespace ralgo
{
	//Функция ограничения в заданном диапазоне.
	template <class T> T clamp(T val, T lo, T hi)
	{
		return val < lo ? lo : val > hi ? hi : val;
	}

	//Функция ограничения вне заданного диапазона.
	template <class T> T rlamp(T val, T zone)
	{
		return val < -zone ? val : val > zone ? val : val < 0 ? -zone : zone;
	}

	template <class T> constexpr T abs(const T &x)
	{
		return x > 0 ? x : -x;
	}

	template <typename A> constexpr A max(A a, A b)
	{
		return a > b ? a : b;
	}
	
	template <typename A> constexpr A min(A a, A b)
	{
		return a < b ? a : b;
	}
	
	template <typename A> constexpr void swap(A& a, A& b)
	{
		A tmp = a; a = b; b = tmp;
	}

	template <class A, class B> constexpr A sign(const A& mag, const B& sign)
	{
		return sign >= 0 ? abs(mag) : -abs(mag);
	}

	template <class A> constexpr A sqr(const A& a)
	{
		return a * a;
	}
} // namespace ralgo

#endif
