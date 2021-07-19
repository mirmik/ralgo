#ifndef RALGO_HEIMER_STEPCTR_APPLIER
#define RALGO_HEIMER_STEPCTR_APPLIER

#include <ralgo/heimer2/stepctr.h>
#include <ralgo/heimer2/axis_state.h>

struct stepctr_applier 
{
	struct stepctr * controlled_stepctr;
	struct axis_state * state;

	float gain;
};

__BEGIN_DECLS

void stepctr_applier_init(
	struct stepctr_applier * applier,
	struct stepctr * stepctr,
	struct axis_state * state
) 
{
	applier->controlled_stepctr = stepctr;
	applier->state = state; 
}

__END_DECLS

#endif