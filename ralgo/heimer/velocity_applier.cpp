#include <ralgo/heimer/velocity_applier.h>
#include <ralgo/log.h>
#include <igris/math.h>
#include <igris/util/numconvert.h>

using namespace heimer;

velocity_applier::velocity_applier()
	: signal_processor(0, 1)
{}

velocity_applier::velocity_applier(
    const char * name,
    robo::fixed_frequency_stepper_controller * stepctr,
    axis_state * state
)
	: signal_processor(0, 1)
{
	init(name, stepctr, state);
}

velocity_applier::velocity_applier(
    const char * name,
    robo::fixed_frequency_stepper_controller * stepctr
)
	: signal_processor(0, 1)
{
	set_name(name);
	controlled_velset = stepctr;
	controlled_velget = stepctr;
	controlled_posget = stepctr;
	this->stepctr = stepctr;
}

void velocity_applier::init(
    const char * name,
    robo::fixed_frequency_stepper_controller * stepctr,
    axis_state * state)
{
	set_name(name);
	controlled_velset = stepctr;
	controlled_velget = stepctr;
	controlled_posget = stepctr;
	this->state = state;
	state->attach_listener(this);
	this->stepctr = stepctr;
}

void velocity_applier::deinit()
{
	signal_processor::deinit();
	state->deattach_listener(this);
}

int velocity_applier::serve(disctime_t time)
{
	disctime_t delta = time - last_time;
	position_t errpos = state->ctrpos - state->feedpos;
	velocity_t used_compkoeff = 
		(ABS(errpos) < 1e-2 && ABS(state->ctrvel) == 0) ?
		 compkoeff_hard : compkoeff;

	if (deviation_error_limit && ABS(errpos) > deviation_error_limit)
	{
		controlled_velset->set_velocity(0);

		char str[56];
		sprintf(str, "position deviation error : mnemo:%s", name().data());
		ralgo::warn(str);

		interrupt(time, true);
		return SIGNAL_PROCESSOR_RETURN_RUNTIME_ERROR;
	}

	compspd = state->ctrvel + used_compkoeff * errpos * delta;
	velocity_t impulses_per_sec = compspd * gear;

	/*if (!is_active())
	{
		if (errpos < 1e-6)
		{
			controlled_velset->set_velocity(0);
		}
	}
	else
	{*/
		controlled_velset->set_velocity(impulses_per_sec);
	//}

	last_time = time;
	return 0;
}


int velocity_applier::feedback(disctime_t time)
{
	(void) time;
	state->feedpos = controlled_posget->feedback_position() / gear;
	state->feedvel = controlled_velget->feedback_velocity() / gear;
	return 0;
}

int velocity_applier::info(char * ans, int anslen)
{
	static char stepctr_info[256];
	stepctr->info(stepctr_info, 256);

	nos::format_buffer(ans,
	                   "listen: {} \r\n"
	                   "feedpos: {} \r\n"
	                   "stepctr_info: \r\n{}",
	                   state->name,
	                   controlled_posget->feedback_position(),
	                   stepctr_info
	                  );

	return 0;
}

int velocity_applier::bind(int argc, char ** argv, char * output, int outmax)
{
	if (argc != 1)
	{
		snprintf(output, outmax, "Can't bind %d symbols for %d _dim velctr", argc, 1);
		return -1;
	}

	signal_head * sig = signal_get_by_name(argv[0]);

	if (!sig)
	{
		snprintf(output, outmax, "Wrong signal name '%s' (type 'siglist' for display)", argv[0]);
		return -1;
	}

	if (sig->type != SIGNAL_TYPE_AXIS_STATE )
	{
		snprintf(output, outmax, "Wrong signal type. name:(%s)", sig->name);
		return -1;
	}

	state = static_cast<axis_state*>(sig);
	state->attach_listener(this);

	return 0;
}

void velocity_applier::set_gear(float gear)
{
	this->gear = gear;
}

int velocity_applier::command(int argc, char ** argv, char * output, int outmax)
{
	int status = ENOENT;

	if (strcmp("bind", argv[0]) == 0)
		status = bind(argc - 1, argv + 1, output, outmax);

	if (strcmp("setgear", argv[0]) == 0)
	{
		set_gear(atof32(argv[1], nullptr));
		return 0;
	}

	if (strcmp("info", argv[0]) == 0)
		status = info(output, outmax);

	return status;
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
