#ifndef RALGO_HEIMER2_STEPCTR_H
#define RALGO_HEIMER2_STEPCTR_H

#include <stdint.h>
#include <math.h>
#include <igris/compiler.h>

#include <ralgo/disctime.h>

#define STEPCTR_OVERRUN -22

struct stepctr_controller
{
	void (*dec)(void*);
	void (*inc)(void*);
	void * incdec_priv;

	int64_t units_in_step;
	int64_t units_in_step_triggered;

	int64_t control_pos;
	int64_t virtual_pos;
};

__BEGIN_DECLS

void stepctr_controller_init(
	struct stepctr_controller * ctr,
	int64_t units_in_step,
	float trigger_level
)
{
	ctr -> units_in_step = units_in_step;
	ctr -> units_in_step_triggered = trigger_level * units_in_step;

	ctr->control_pos = 0;
	ctr->virtual_pos = 0; 
}

void stepctr_controller_set_position(struct stepctr_controller * ctr, int64_t pos) 
{
	ctr->control_pos = ctr->virtual_pos = pos;
}

int stepctr_controller_shift(struct stepctr_controller * ctr, int64_t shift)
{
	if (abs(shift) >= ctr->units_in_step)
		return STEPCTR_OVERRUN;

	ctr->virtual_pos += shift;
	int64_t diffpos = ctr->virtual_pos - ctr->control_pos;

	if (diffpos > 0)
	{
		if (diffpos > ctr->units_in_step_triggered)
		{
			ctr->inc(ctr->incdec_priv);
			ctr->control_pos += ctr->units_in_step;
		}
	}

	else
	{
		if (diffpos < -ctr->units_in_step_triggered)
		{
			ctr->dec(ctr->incdec_priv);
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

__END_DECLS

#endif