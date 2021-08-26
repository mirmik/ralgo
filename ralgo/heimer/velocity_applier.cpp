#include <ralgo/heimer/velocity_applier.h>
#include <ralgo/log.h>
#include <igris/math.h>

using namespace heimer;

velocity_applier::velocity_applier(
    const char * name,
    robo::fixed_frequency_stepper_controller * stepctr,
    axis_state * state
)
	: signal_processor(name, 0, 1)
{
	controlled_velset = stepctr;
	controlled_velget = stepctr;
	controlled_posget = stepctr;
	this->state = state;

	state->attach_listener(this);
}

void velocity_applier::deinit()
{
	signal_processor::deinit();
	state->deattach_listener(this);
}

int velocity_applier::serve(disctime_t time)
{
	position_t errpos = state->ctrpos - state->feedpos;

	if (deviation_error_limit && ABS(errpos) > deviation_error_limit)
	{
		controlled_velset->set_velocity(0);

		char str[56];
		sprintf(str, "position deviation error : mnemo:%s", name().data());
		ralgo::warn(str);

		interrupt(time, true);
		return SIGNAL_PROCESSOR_RETURN_RUNTIME_ERROR;
	}

	disctime_t delta = time - last_time;

	compspd = state->ctrvel + compkoeff * errpos * delta;
	impulses_per_disc = compspd * gain;
	controlled_velset->set_velocity(impulses_per_disc);

	last_time = time;
	return 0;
}


int velocity_applier::feedback(disctime_t time)
{
	(void) time;
	state->feedpos = controlled_posget->feedback_position() / gain;
	state->feedvel = controlled_velget->feedback_velocity() / gain;
	return 0;
}

int velocity_applier::command(int, char **, char *, int) 
{
	return 0;
}

signal_head * velocity_applier::iterate_left(signal_head *) 
{
	return NULL;
}

signal_head * velocity_applier::iterate_right(signal_head * iter) 
{
	if (iter) return NULL;
	else return state;
}
