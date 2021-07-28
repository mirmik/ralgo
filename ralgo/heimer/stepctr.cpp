#include <ralgo/heimer/stepctr.h>

#include <math.h>
#include <igris/math.h>
#include <ralgo/disctime.h>

using namespace heimer;

void stepctr_controller::init(
	void (*set_quaddgen_state)(void*, uint8_t),
	void * priv,
	int64_t units_in_step,
	float trigger_level
)
{
	this->set_quaddgen_state = set_quaddgen_state;
	this->incdec_priv = priv;

	this->units_in_step = units_in_step;
	this->units_in_step_triggered = trigger_level * units_in_step;

	control_pos = 0;
	virtual_pos = 0;

	state = 0; 
}

void stepctr_controller::set_position(int64_t pos) 
{
	control_pos = pos;
	virtual_pos = pos;
}

void stepctr_controller::inc() 
{
	state = state == 3 ? 0 : state + 1;
	set_quaddgen_state(incdec_priv, state);
}

void stepctr_controller::dec() 
{
	state = state == 0 ? 3 : state - 1;
	set_quaddgen_state(incdec_priv, state);
}

int stepctr_controller::shift(int64_t shift)
{
	if (ABS(shift) >= units_in_step)
		return STEPCTR_OVERRUN;

	virtual_pos += shift;
	int64_t diffpos = virtual_pos - control_pos;

	if (diffpos > 0)
	{
		if (diffpos > units_in_step_triggered)
		{
			inc();
			control_pos += units_in_step;
		}
	}

	else
	{
		if (diffpos < -units_in_step_triggered)
		{
			dec();
			control_pos -= units_in_step;
		}
	}

	return 0;
}

int stepctr_controller::speed_apply(
	float speed, 
	float delta 
		// дискретное время дано с плавающей точкой,
		// чтобы можно было передавать интервалы времени
		// меньше disctime

) 
{
	int64_t arg = speed * delta;
	return shift(arg);
}