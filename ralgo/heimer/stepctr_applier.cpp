#include <ralgo/heimer/stepctr_applier.h>
#include <igris/math.h>

using namespace heimer;

stepctr_applier::stepctr_applier(
    const char * name,
    struct stepctr_controller * stepctr,
    struct axis_state * state
)
	: signal_processor(name, 0, 1)
{
	controlled_stepctr = stepctr;
	this->state = state;

	state->attach_listener(this);
}

void stepctr_applier::deinit()
{
	signal_processor::deinit();
	state->deattach_listener(this);
}

int stepctr_applier::serve(disctime_t time)
{
	(void) time;

	int64_t errpos = state->ctrpos - state->feedpos;

	if (ABS(errpos) > deviation_error_limit)
	{
		controlled_stepctr->set_speed(0);

		//char str[56];
		//sprintf(str, "position deviation error : mnemo:%s", parent::mnemo());
		//ralgo::warn(str);

		//force_stop_interrupt_args msg("position deviation error");
		//parent::throw_interrupt(&msg);
	}

	// Скорость вычисляется как
	// сумма уставной скорости на
	// Возможно, в этом выражение должно быть время.
	float compspd = state->ctrvel + compkoeff * errpos;
	controlled_stepctr->set_speed(compspd);

	return 0;
}


int stepctr_applier::feedback(disctime_t time)
{
	(void) time;
	return 0;
}

int stepctr_applier::command(int, char **, char *, int) 
{
	return 0;
}

signal_head * stepctr_applier::iterate_left(signal_head *) 
{
	return NULL;
}

signal_head * stepctr_applier::iterate_right(signal_head * iter) 
{
	if (iter) return NULL;
	else return state;
}
