#include <ralgo/heimer/stepctr_applier.h>
#include <ralgo/log.h>
#include <igris/math.h>

using namespace heimer;

stepctr_applier::stepctr_applier(
    const char * name,
    robo::fixed_frequency_stepper_controller * stepctr,
    axis_state * state
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
	position_t errpos = state->ctrpos - state->feedpos;

	if (deviation_error_limit && ABS(errpos) > deviation_error_limit)
	{
		controlled_stepctr->set_speed(0);

		char str[56];
		sprintf(str, "position deviation error : mnemo:%s", name().data());
		ralgo::warn(str);

		interrupt(time, true);
		return SIGNAL_PROCESSOR_RETURN_RUNTIME_ERROR;
	}

	disctime_t delta = time - last_time;

	float compspd = state->ctrvel + compkoeff * errpos * delta;
	controlled_stepctr->set_speed(compspd * gain);

	last_time = time;
	return 0;
}


int stepctr_applier::feedback(disctime_t time)
{
	(void) time;
	state->feedpos = controlled_stepctr->feedback_position() / gain;
	state->feedvel = controlled_stepctr->feedback_speed() / gain;
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
