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
		float koeff;
		T state;

	public:
		aperiodic_filter(float _koeff) : koeff(_koeff) {}

		T operator()(const T & in) override 
		{
			state += (in - state) * koeff;
			return state;
		}

		void reset(T val) { state = val; }
	};
}

#endif