#include <ralgo/robo/stepper_controller.h>

#include <math.h>
#include <igris/math.h>
#include <ralgo/disctime.h>

using namespace robo;

stepper_controller::stepper_controller( robo::stepper * stepper )
{
	this->stepper = stepper;

	//this->units_in_step = units_in_step;
	//this->units_in_step_triggered = trigger_level * units_in_step;

	control_pos = 0;
	virtual_pos = 0;

	state = 0;
}

void stepper_controller::set_steps_position(position_t pos)
{
	control_pos = pos * units_in_step;
	virtual_pos = pos * units_in_step;
}

int stepper_controller::shift(int64_t shift)
{
	if (ABS(shift) >= units_in_step)
		return STEPCTR_OVERRUN;

	virtual_pos += shift;
	int64_t diffpos = virtual_pos - control_pos;

	if (diffpos > 0)
	{
		if (diffpos > units_in_step_triggered)
		{
			stepper->inc();
			control_pos += units_in_step;
		}
	}

	else
	{
		if (diffpos < -units_in_step_triggered)
		{
			stepper->dec();
			control_pos -= units_in_step;
		}
	}

	return 0;
}

int stepper_controller::speed_apply(
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

void fixed_frequency_stepper_controller::constant_frequency_serve()
{
	int sts = shift(current_shift);

	if (sts) 
	{
		interrupt_handle(interrupt_priv, sts);
	}
}

fixed_frequency_stepper_controller::fixed_frequency_stepper_controller(
	robo::stepper * stepper
)
	: stepper_controller(stepper)
{

}

void fixed_frequency_stepper_controller::set_speed(float speed)
{
	this->speed = speed;
	this->current_shift = frequency * speed;
}