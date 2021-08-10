#ifndef RALGO_ROBO_QUADGEN_H
#define RALGO_ROBO_QUADGEN_H

#include <ralgo/robo/stepper.h>

namespace robo
{
	class quadgen : public robo::stepper
	{
		int8_t state = 0;

	public:
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

		virtual void set_state(uint8_t state) = 0;
	};
}

#endif