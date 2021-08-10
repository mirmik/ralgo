#ifndef RALGO_HEIMER_STEPPER_CONTROLLER_H
#define RALGO_HEIMER_STEPPER_CONTROLLER_H

#include <stdint.h>
#include <igris/compiler.h>

#include <ralgo/heimer/heimer_types.h>
#include <ralgo/robo/stepper.h>
#include <ralgo/disctime.h>

#define STEPCTR_OVERRUN -22

namespace robo
{
	/**
		Драйвер шагового двигателя или любого устройства, управляемого шагами.
		Даёт команду на совершения шага изделия в зависимости от установленной скорости.
	*/
	class stepper_controller
	{
		robo::stepper * stepper;

		double ext2steps;

		float trigger_level = 0.75;

		int64_t _control_pos = 0;
		int64_t _virtual_pos = 0;

		uint8_t state = 0;

	protected:
		int64_t units_in_step = 10000;
		int64_t units_in_step_triggered = units_in_step * trigger_level;

	public:
		stepper_controller(robo::stepper * stepper);

		void set_steps_position(position_t pos);
		void set_position(position_t pos);

		int shift(int64_t shift);

		// дискретное время дано с плавающей точкой,
		// чтобы можно было передавать интервалы времени
		// меньше disctime
		int speed_apply(
		    float speed,
		    float delta
		);

		void set_trigger_level(float trigger_level)
		{
			this->trigger_level = trigger_level;
		}

		void set_units_in_step(int64_t ups)
		{
			units_in_step = ups;
			units_in_step_triggered = ups * trigger_level;
		}

		int64_t control_pos() { return _control_pos; }
		int64_t virtual_pos() { return _virtual_pos; }
	};

	class fixed_frequency_stepper_controller : public stepper_controller
	{
		float   speed_to_shift = 1;
		int64_t current_shift = 0;

		float freq = 1;
		float gain = 1;

		void(*interrupt_handle)(void*, int);
		void * interrupt_priv;

	public:
		fixed_frequency_stepper_controller(robo::stepper * stepper);

		void set_frequency(float freq)
		{
			this->freq = freq;
			evaluate_speed_to_shift();
		}

		void set_gain(float gain)
		{
			this->gain = gain;
			evaluate_speed_to_shift();
		}

		void set_interrupt_handler(void(*handle)(void*, int), void * arg)
		{
			this->interrupt_handle = handle;
			this->interrupt_priv = arg;
		}

		void set_speed(float speed);

		void constant_frequency_serve();

	private:
		void evaluate_speed_to_shift()
		{
			speed_to_shift = freq * units_in_step * gain / discrete_time_frequency();
		}
	};
}

#endif