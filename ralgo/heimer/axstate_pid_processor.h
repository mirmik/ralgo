#ifndef RALGO_HEIMER_AXSTATE_PID_PROCESSOR_H
#define RALGO_HEIMER_AXSTATE_PID_PROCESSOR_H

#include <ralgo/heimer/axstate_signal_processor.h>
#include <ralgo/heimer/scalar_signal.h>

namespace heimer
{
	class axstate_pid_processor : public signal_processor
	{
		axis_state * state;
		scalar_signal * target;

		double compcoeff;
		disctime_t last_serve_time;

	public:
		axstate_pid_processor(const char * name);

		int feedback(disctime_t time) override;
		int serve(disctime_t time) override;
		int command(int argc, char ** argv, char * output, int outmax) override;
		void deinit() override;

		void set_right(scalar_signal *);
		void set_left(axis_state *);
		void set_compcoeff(double coeff);

		signal_head * iterate_left(signal_head * iter) override;
		signal_head * iterate_right(signal_head * iter) override;

		void on_activate(disctime_t time) override;
	};
}

#endif