#ifndef RALGO_UTIL_CLAMPER_H
#define RALGO_UTIL_CLAMPER_H

namespace ralgo
{
	template <class T>
	class clamper
	{
		T lo = T();
		T hi = T();
		bool enabled = false;

	public:
		clamper() = default;
		
		void set_limits(const T& lo, const T& hi) 
		{
			this->lo = lo;
			this->hi = hi;
		}

		void enable(bool en) 
		{
			enabled = en;
		}

		T operator() (const T & x) 
		{
			if (!enabled)
				return x;

			return std::clamp(x, lo, hi);
		}
	};
}

#endif
