#ifndef RALGO_HEIMER_AXSTATE_PID_PROCESSOR_H
#define RALGO_HEIMER_AXSTATE_PID_PROCESSOR_H

#include <ralgo/heimer/axis_state.h> 

namespace heimer 
{
	class axstate_pid_processor 
	{
		axis_state * left;
		axis_state * right;

	public:
		axstate_pid_processor(const char * name);

		int feedback(disctime_t time) override;
		int serve(disctime_t time) override;
		int command(int argc, char ** argv, char * output, int outmax) override;
		void deinit() override;
		signal_head * iterate_left(signal_head *) override;
		signal_head * iterate_right(signal_head *) override;	

		void set_target(position_t target);
	}
}

#endif