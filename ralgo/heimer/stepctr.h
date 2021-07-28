#ifndef RALGO_HEIMER2_STEPCTR_H
#define RALGO_HEIMER2_STEPCTR_H

#include <stdint.h>
#include <igris/compiler.h>

#define STEPCTR_OVERRUN -22

/**
	Драйвер шагового двигателя или любого устройства, управляемого шагами.
	Даёт команду на совершения шага изделия в зависимости от установленной скорости. 
*/
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

public:
	void init(
	    void (*set_quaddgen_state)(void*, uint8_t),
	    void * priv,
	    int64_t units_in_step,
	    float trigger_level
	);

	void set_position(int64_t pos);

	void inc();

	void dec();

	int shift(int64_t shift);

	// дискретное время дано с плавающей точкой,
	// чтобы можно было передавать интервалы времени
	// меньше disctime
	int speed_apply(
	    float speed,
	    float delta
	);

	void set_speed(float speed)
	{
		setted_speed = speed;

		if (fixed_frequency)
		{
		}
		else
		{
		}
	}
};

#endif