#ifndef RALGO_HEIMER2_STEPCTR_H
#define RALGO_HEIMER2_STEPCTR_H

#include <stdint.h>
#include <igris/compiler.h>

#define STEPCTR_OVERRUN -22

struct stepctr_controller
{
	void (*set_quaddgen_state)(void*, uint8_t);
	void * incdec_priv;

	int64_t units_in_step;
	int64_t units_in_step_triggered;

	int64_t control_pos;
	int64_t virtual_pos;

	uint8_t state;

	/////// Второй уровень работа с установленным состоянием.
	float fixed_frequency;

	int64_t current_shift;
	float setted_speed;
};

__BEGIN_DECLS

void stepctr_controller_init(
    struct stepctr_controller * ctr,
    void (*set_quaddgen_state)(void*, uint8_t),
    void * priv,
    int64_t units_in_step,
    float trigger_level
);

void stepctr_controller_set_position(struct stepctr_controller * ctr, int64_t pos);

void stepctr_controller_inc(struct stepctr_controller * ctr);

void stepctr_controller_dec(struct stepctr_controller * ctr);

int stepctr_controller_shift(struct stepctr_controller * ctr, int64_t shift);

int stepctr_controller_speed_apply(struct stepctr_controller * ctr,
                                   float speed,
                                   float delta
                                   // дискретное время дано с плавающей точкой,
                                   // чтобы можно было передавать интервалы времени
                                   // меньше disctime

                                  );

static inline
void stepctr_controller_set_speed(struct stepctr_controller * ctr, float speed)
{
	ctr->setted_speed = speed;

	if (ctr->fixed_frequency)
	{

	}
	else
	{
	}
}

__END_DECLS

#endif