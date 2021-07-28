#include <ralgo/heimer/stepctr_applier.h>
#include <igris/math.h>

void stepctr_applier::init(
    const char * name,
    struct stepctr_controller * stepctr,
    struct axis_state * state
)
{
	signal_processor::init(name);
	controlled_stepctr = stepctr;
	this->state = state;

	state->get();
}

void stepctr_applier::deinit()
{
	signal_processor::deinit();
	state->put();
}

int stepctr_applier::serve(disctime_t time)
{
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
	float compspd = state->ctrvel + compkoeff * errpos;
	controlled_stepctr->set_speed(compspd);

	return 0;
}


int stepctr_applier::feedback(disctime_t time)
{
	return 0;
}

int stepctr_applier::command(int argc, char ** argv, char * output, int outmax) 
{
	return 0;
}

struct signal_head * stepctr_applier::iterate_left(struct signal_head *) 
{
	return NULL;
}

struct signal_head * stepctr_applier::iterate_right(struct signal_head * iter) 
{
	if (iter) return NULL;
	else return state;
}
