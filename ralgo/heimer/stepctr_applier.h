#ifndef RALGO_HEIMER_STEPCTR_APPLIER
#define RALGO_HEIMER_STEPCTR_APPLIER

#include <ralgo/heimer/stepctr.h>
#include <ralgo/heimer/axis_state.h>
#include <ralgo/heimer/signal_processor.h>

namespace heimer
{
	class stepctr_applier : public signal_processor
	{
		stepctr_controller * controlled_stepctr;
		axis_state * state;

		int64_t deviation_error_limit;
		float compkoeff; /// Коэффициент комплементарного фильтра.
		float gain;

	public:
		int feedback(disctime_t time) override;
		int serve(disctime_t time) override;
		int command(int argc, char ** argv, char * output, int outmax) override;
		void deinit() override;
		signal_head * iterate_left(signal_head *) override;
		signal_head * iterate_right(signal_head *) override;

		void init(
		    const char * name,
		    struct stepctr_controller * stepctr,
		    struct axis_state * state
		);
	};
}

#endif