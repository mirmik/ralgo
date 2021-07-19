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
);

void stepctr_controller_set_position(struct stepctr_controller * ctr, int64_t pos);

int stepctr_controller_shift(struct stepctr_controller * ctr, int64_t shift);

int stepctr_controller_speed_apply(struct stepctr_controller * ctr, 
	float speed, 
	float delta 
		// дискретное время дано с плавающей точкой,
		// чтобы можно было передавать интервалы времени
		// меньше disctime

);

__END_DECLS

#endif