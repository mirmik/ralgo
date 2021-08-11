#ifndef RALGO_HEIMER_STEPCTR_APPLIER
#define RALGO_HEIMER_STEPCTR_APPLIER

#include <ralgo/robo/stepper_controller.h>
#include <ralgo/heimer/phase_signals.h>
#include <ralgo/heimer/signal_processor.h>

namespace heimer
{
	class stepctr_applier : public signal_processor
	{
		robo::fixed_frequency_stepper_controller * controlled_stepctr;
		axis_state * state;

		position_t deviation_error_limit = 0;
		float compkoeff = 0; /// Коэффициент комплементарного фильтра.
		float gain = 1;
		disctime_t last_time;

	public:
	// debug
		velocity_t compspd;
		velocity_t impulses_per_disc;

	public:
		stepctr_applier(
		    const char * name,
		    robo::fixed_frequency_stepper_controller * stepctr,
		    axis_state * state
		);

		void set_gain(float gain) { this->gain = gain; }
		void set_compkoeff(float ck) { this->compkoeff = ck; }

		void set_compkoeff_timeconst(float T) { this->compkoeff = 1. / discrete_time_frequency() / T; }

		int feedback(disctime_t time) override;
		int serve(disctime_t time) override;
		int command(int argc, char ** argv, char * output, int outmax) override;
		void deinit() override;
		signal_head * iterate_left(signal_head *) override;
		signal_head * iterate_right(signal_head *) override;
	};
}

#endif