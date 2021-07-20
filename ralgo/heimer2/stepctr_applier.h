#ifndef RALGO_HEIMER_STEPCTR_APPLIER
#define RALGO_HEIMER_STEPCTR_APPLIER

#include <ralgo/heimer2/stepctr.h>
#include <ralgo/heimer2/axis_state.h>
#include <ralgo/heimer2/signal_processor.h>

struct stepctr_applier
{
	struct signal_processor sigproc;

	struct stepctr_controller * controlled_stepctr;
	struct axis_state * state;

	int64_t deviation_error_limit;
	float compkoeff; /// Коэффициент комплементарного фильтра.
	float gain;
};

__BEGIN_DECLS

void stepctr_applier_init(
    struct stepctr_applier * applier,
    const char * name,
    struct stepctr_controller * stepctr,
    struct axis_state * state
);

void stepctr_applier_deinit(struct stepctr_applier * applier);

void stepctr_applier_serve(struct signal_processor * proc, disctime_t time);

__END_DECLS

#endif