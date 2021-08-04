#include <ralgo/heimer/axis_stub_processor.h>
#include <ralgo/heimer/sigtypes.h>

heimer::axis_stub_processor::axis_stub_processor(const char* name)
	: signal_processor(name, 1, 0)
{

}

int heimer::axis_stub_processor::feedback(disctime_t)
{
	_axstate->feedpos = pos;
	_axstate->feedvel = vel;

	return 0;
}

int heimer::axis_stub_processor::serve(disctime_t time)
{
	if (_axstate == nullptr || !is_active())
		return SIGNAL_PROCESSOR_RETURN_NOT_ACTIVE;

	pos = _axstate->ctrpos;
	vel = _axstate->ctrvel;

	if (_apply_speed_mode)
	{
		disctime_t delta = time - lasttime;
		pos = pos + vel * delta;
	}

	lasttime = time;
	return 0;
}

static inline
int bind(heimer::axis_stub_processor * axctr, int argc, char ** argv, char * output, int outmax)
{
	if (argc < 1)
	{
		snprintf(output, outmax, "Need argument");
		return -1;
	}

	heimer::signal_head * sig = heimer::signal_get_by_name(argv[0]);

	if (!sig)
	{
		snprintf(output, outmax, "Wrong signal name '%s' (type 'siglist' for display)", argv[0]);
		return -1;
	}

	if (sig->type != SIGNAL_TYPE_AXIS_STATE)
	{
		snprintf(output, outmax, "Wrong signal type. name:(%s)", sig->name);
		return -1;
	}

	axctr->bind(static_cast<heimer::axis_state *>(sig));

	return 0;
}


static inline
int info(heimer::axis_stub_processor * axctr, int, char **, char * output, int outmax)
{
	snprintf(output, outmax, "is_active: %d\r\n", axctr->is_active());
	return 0;
}

int heimer::axis_stub_processor::command(int argc, char ** argv, char * output, int outmax)
{
	int status = ENOENT;

	if (strcmp("bind", argv[0]) == 0)
		status = ::bind(this, argc - 1, argv + 1, output, outmax);

	if (strcmp("info", argv[0]) == 0)
		status = ::info(this, argc - 1, argv + 1, output, outmax);

	return status;
}

void heimer::axis_stub_processor::deinit()
{
	if (_axstate)
		_axstate->deattach_listener(this);

	_axstate = nullptr;
}

heimer::signal_head * heimer::axis_stub_processor::iterate_left(heimer::signal_head *)
{
	return NULL;
}

heimer::signal_head * heimer::axis_stub_processor::iterate_right(heimer::signal_head * iter)
{
	if (iter == NULL)
		return _axstate;
	else
		return NULL;
}

void heimer::axis_stub_processor::bind(heimer::axis_state * iter)
{
	_axstate = iter;
	_axstate->attach_listener(this);
}

void heimer::axis_stub_processor::apply_speed_mode(bool en)
{
	_apply_speed_mode = en;
}

void heimer::axis_stub_processor::on_activate(disctime_t time)
{
	lasttime = time;
}