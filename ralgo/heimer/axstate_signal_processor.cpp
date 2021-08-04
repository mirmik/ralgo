#include <ralgo/heimer/axstate_signal_processor.h>
#include <ralgo/log.h>
#include <ralgo/heimer/sigtypes.h>

heimer::signal_head * heimer::axstate_signal_processor::iterate_left(signal_head * iter)
{
	if (!_leftside)
	{
		ralgo::warn("leftside is not allocated");
		return nullptr;
	}

	if (iter == NULL)
		return *_leftside;

	heimer::axis_state ** it = _leftside;
	for (; it != _leftside + leftdim() - 1  ; ++it)
	{
		if (*it == iter)
		{
			it++;
			return *it;
		}
	}

	return NULL;
}

heimer::signal_head * heimer::axstate_signal_processor::iterate_right(signal_head * iter)
{
	if (!_rightside)
	{
		ralgo::warn("rightside is not allocated");
		return nullptr;
	}

	if (iter == NULL)
		return *_rightside;

	heimer::axis_state ** it = _rightside;
	for (; it != _rightside + rightdim() - 1  ; ++it)
	{
		if (*it == iter)
		{
			it++;
			return *it;
		}
	}

	return NULL;
}

heimer::axstate_signal_processor::axstate_signal_processor(const char * name, int ldim, int rdim)
	: heimer::signal_processor(name, ldim, rdim)
{

}


void heimer::axstate_signal_processor::attach_leftside_table(axis_state ** table)
{
	_leftside = table;
}

void heimer::axstate_signal_processor::attach_rightside_table(axis_state ** table)
{
	_rightside = table;
}

void heimer::axstate_signal_processor::set_leftside(heimer::axis_state ** arr)
{
	if (!_leftside)
	{
		ralgo::warn("leftside is not allocated");
		return;
	}

	for (int i = 0; i < leftdim(); ++i)
	{
		_leftside[i] = arr[i];
		_leftside[i] -> attach_possible_controller(this);
	}
}

void heimer::axstate_signal_processor::set_rightside(heimer::axis_state ** arr)
{
	if (!_rightside)
	{
		ralgo::warn("rightside is not allocated");
		return;
	}

	for (int i = 0; i < rightdim(); ++i)
	{
		_rightside[i] = arr[i];
		_rightside[i] -> attach_listener(this);
	}
}


int heimer::axstate_signal_processor_bindleft(axstate_signal_processor * axctr, int argc, char ** argv, char * output, int outmax)
{
	if (argc != axctr->leftdim())
	{
		snprintf(output, outmax, "Can't bind %d symbols for %d _dim axisctr", argc, axctr->leftdim());
		return -1;
	}

	{
		axis_state * arr[argc];

		for (int i = 0; i < argc; ++i)
		{
			signal_head * sig = signal_get_by_name(argv[i]);

			if (!sig)
			{
				snprintf(output, outmax, "Wrong signal name '%s ' (type 'siglist' for display)", argv[i]);
				return -1;
			}

			if (sig->type != SIGNAL_TYPE_AXIS_STATE)
			{
				snprintf(output, outmax, "Wrong signal type. name:(%s)", sig->name);
				return -1;
			}

			arr[i] = static_cast<axis_state *>(sig);
		}

		axctr->set_leftside(arr);
	}

	return 0;
}

int heimer::axstate_signal_processor_bindright(axstate_signal_processor * axctr, int argc, char ** argv, char * output, int outmax)
{
	if (argc != axctr->rightdim())
	{
		snprintf(output, outmax, "Can't bind %d symbols for %d _dim axisctr", argc, axctr->rightdim());
		return -1;
	}

	{
		axis_state * arr[argc];

		for (int i = 0; i < argc; ++i)
		{
			signal_head * sig = signal_get_by_name(argv[i]);

			if (!sig)
			{
				snprintf(output, outmax, "Wrong signal name '%s' (type 'siglist' for display)", argv[i]);
				return -1;
			}

			if (sig->type != SIGNAL_TYPE_AXIS_STATE)
			{
				snprintf(output, outmax, "Wrong signal type. name:(%s)", sig->name);
				return -1;
			}

			arr[i] = static_cast<axis_state *>(sig);
		}

		axctr->set_rightside(arr);
	}

	return 0;
}
