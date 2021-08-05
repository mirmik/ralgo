/** @filr */

#ifndef RALGO_HEIMER_AXIS_STUB_PROCESSOR_H
#define RALGO_HEIMER_AXIS_STUB_PROCESSOR_H

#include <ralgo/heimer/signal_processor.h>
#include <ralgo/heimer/phase_signals.h>

namespace heimer
{
	class axis_stub_processor : public signal_processor
	{
	private:
		axis_state * _axstate = nullptr;
		bool _apply_speed_mode = false;

	public:
		position_t pos;
		velocity_t vel;

		disctime_t lasttime;

	public:
		axis_stub_processor(const char* name);

		int feedback(disctime_t time) override;
		int serve(disctime_t time) override;
		int command(int argc, char ** argv, char * output, int outmax) override;
		void deinit() override;
		signal_head * iterate_left(signal_head *) override;
		signal_head * iterate_right(signal_head *) override;

		void apply_speed_mode(bool en);

		void bind(axis_state * axstate);
		void on_activate(disctime_t time) override; 
	};
}

#endif