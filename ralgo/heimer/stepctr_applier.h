#ifndef RALGO_HEIMER_STEPCTR_APPLIER
#define RALGO_HEIMER_STEPCTR_APPLIER

#include <ralgo/heimer/stepctr.h>
#include <ralgo/heimer/axis_state.h>
#include <ralgo/heimer/signal_processor.h>

class stepctr_applier : public signal_processor
{
	stepctr_controller * controlled_stepctr;
	axis_state * state;

	int64_t deviation_error_limit;
	float compkoeff; /// Коэффициент комплементарного фильтра.
	float gain;

public:
	void feedback(disctime_t time) override;
	void serve(disctime_t time) override;
	int command(int argc, char ** argv, char * output, int outmax) override;
	void deinit() override;
	struct signal_head * iterate_left(struct signal_head *) override;

	void init(
	    const char * name,
	    struct stepctr_controller * stepctr,
	    struct axis_state * state
	);
};

#endif