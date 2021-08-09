#ifndef RALGO_HEIMER_STEPCTR_APPLIER
#define RALGO_HEIMER_STEPCTR_APPLIER

#include <ralgo/robo/stepper_controller.h>
#include <ralgo/heimer/phase_signals.h>
#include <ralgo/heimer/signal_processor.h>

namespace heimer
{
	class stepctr_applier : public signal_processor
	{
		robo::stepper_controller * controlled_stepctr;
		axis_state * state;

		int64_t deviation_error_limit;
		float compkoeff; /// Коэффициент комплементарного фильтра.
		float gain;

	public:
		stepctr_applier(
		    const char * name,
		    robo::stepper_controller * stepctr,
		    axis_state * state
		);

		int feedback(disctime_t time) override;
		int serve(disctime_t time) override;
		int command(int argc, char ** argv, char * output, int outmax) override;
		void deinit() override;
		signal_head * iterate_left(signal_head *) override;
		signal_head * iterate_right(signal_head *) override;
	};
}

#endif