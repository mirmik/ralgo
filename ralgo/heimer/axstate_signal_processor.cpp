#include <ralgo/heimer/axstate_signal_processor.h>
#include <ralgo/log.h>

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
	for (; it != _leftside + _leftdim - 1  ; ++it)
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
	for (; it != _rightside + _rightdim - 1  ; ++it)
	{
		if (*it == iter)
		{
			it++;
			return *it;
		}
	}

	return NULL;
}

heimer::axstate_signal_processor::axstate_signal_processor(const char * name)
	: heimer::signal_processor(name)
{

}


void heimer::axstate_signal_processor::attach_leftside_table(axis_state ** table, int dim)
{
	_leftdim = dim;
	_leftside = table;

	/*if (_leftside)
		for (int i = 0; i < dim; ++i)
		{
			_leftside[i] = nullptr;
		}*/
}

void heimer::axstate_signal_processor::attach_rightside_table(axis_state ** table, int dim)
{
	_rightdim = dim;
	_rightside = table;

	/*if (_rightside)
		for (int i = 0; i < dim; ++i)
		{
			_rightside[i] = nullptr;
		}*/
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