/**
	@file
*/

#ifndef RALGO_FILTER_APERIODIC_FILTER_H
#define RALGO_FILTER_APERIODIC_FILTER_H

#include <ralgo/filter/inout.h>

namespace ralgo
{
	template <class T>
	class aperiodic_filter : public ralgo::inout<T, T>
	{
		T koeff;
		T state;

	public:
		aperiodic_filter() 
		{}

		aperiodic_filter(T _koeff) : koeff(_koeff) 
		{

		}

		T serve(const T & in)
		{
			state += (in - state) * koeff;
			return state;
		}

		T operator()(const T & in) override 
		{
			return serve(in);
		}

		void reset(T val)
		{
			state = val;
		}

		aperiodic_filter& set_koefficient(T delta, T time_constant) 
		{
			koeff = delta / time_constant;
			return *this;
		}
	};
}

#endif
