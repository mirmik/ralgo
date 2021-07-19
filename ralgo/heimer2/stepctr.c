#include <ralgo/heimer2/stepctr.h>

#include <math.h>
#include <igris/math.h>
#include <ralgo/disctime.h>

void stepctr_controller_init(
	struct stepctr_controller * ctr,
	void (*set_quaddgen_state)(void*, uint8_t),
	void * priv,
	int64_t units_in_step,
	float trigger_level
)
{
	ctr -> set_quaddgen_state = set_quaddgen_state;
	ctr -> incdec_priv = priv;

	ctr -> units_in_step = units_in_step;
	ctr -> units_in_step_triggered = trigger_level * units_in_step;

	ctr->control_pos = 0;
	ctr->virtual_pos = 0;

	ctr->state = 0; 
}

void stepctr_controller_set_position(struct stepctr_controller * ctr, int64_t pos) 
{
	ctr->control_pos = pos;
	ctr->virtual_pos = pos;
}

void stepctr_controller_inc(struct stepctr_controller * ctr) 
{
	ctr->state = ctr->state == 3 ? 0 : ctr->state + 1;
	ctr->set_quaddgen_state(ctr->incdec_priv, ctr->state);
}

void stepctr_controller_dec(struct stepctr_controller * ctr) 
{
	ctr->state = ctr->state == 0 ? 3 : ctr->state - 1;
	ctr->set_quaddgen_state(ctr->incdec_priv, ctr->state);
}

int stepctr_controller_shift(struct stepctr_controller * ctr, int64_t shift)
{
	if (ABS(shift) >= ctr->units_in_step)
		return STEPCTR_OVERRUN;

	ctr->virtual_pos += shift;
	int64_t diffpos = ctr->virtual_pos - ctr->control_pos;

	if (diffpos > 0)
	{
		if (diffpos > ctr->units_in_step_triggered)
		{
			stepctr_controller_inc(ctr);
			ctr->control_pos += ctr->units_in_step;
		}
	}

	else
	{
		if (diffpos < -ctr->units_in_step_triggered)
		{
			stepctr_controller_dec(ctr);
			ctr->control_pos -= ctr->units_in_step;
		}
	}

	return 0;
}

int stepctr_controller_speed_apply(struct stepctr_controller * ctr, 
	float speed, 
	float delta 
		// дискретное время дано с плавающей точкой,
		// чтобы можно было передавать интервалы времени
		// меньше disctime

) 
{
	int64_t shift = speed * delta;
	return stepctr_controller_shift(ctr, shift);
}