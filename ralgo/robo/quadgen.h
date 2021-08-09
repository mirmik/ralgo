#ifndef RALGO_ROBO_QUADGEN_H
#define RALGO_ROBO_QUADGEN_H

#include <ralgo/robo/stepper.h>

namespace robo
{
	class quadgen : public robo::stepper
	{
		int64_t counter = 0;
		int8_t state = 0;

	public:
		void set_counter_value(int64_t val) 
		{
			counter = val;
		}

		void set_declared_state(uint8_t declared_state) 
		{
			state = declared_state;
		}

		void inc() override
		{
			++counter;
			state = state == 3 ? 0 : state + 1;
			set_state(state);
		}

		void dec() override
		{
			--counter;
			state = state == 0 ? 3 : state - 1;
			set_state(state);
		}

		int64_t steps_count() override
		{
			return counter;
		}

		virtual void set_state(uint8_t state) = 0;
	};
}

#endif