#ifndef RALGO_HEIMER_STEPPER_CONTROLLER_H
#define RALGO_HEIMER_STEPPER_CONTROLLER_H

#include <stdint.h>
#include <igris/compiler.h>
#include <igris/sync/syslock.h>

#include <ralgo/heimer/heimer_types.h>
#include <ralgo/robo/stepper.h>
#include <ralgo/disctime.h>

#include <ralgo/robo/iposvel.h>

#define STEPCTR_OVERRUN -22

namespace robo
{
	/**
		Драйвер шагового двигателя или любого устройства, управляемого шагами.
		Даёт команду на совершения шага изделия в зависимости от установленной скорости.
	*/
	class stepper_controller : public i_position_feedback
	{
		robo::stepper * stepper;

		float trigger_level = 0.75;

		int64_t _control_pos = 0;
		int64_t _virtual_pos = 0;

	//protected:
	public:
		int64_t units_in_step = (1 << 20);
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
			evaluate();
		}

		virtual void evaluate() {}

		int64_t control_pos() { return _control_pos; }
		int64_t virtual_pos() { return _virtual_pos; }

		double feedback_position() override
		{
			system_lock();
			auto counter_value = stepper->steps_count();
			system_unlock();

			return counter_value;
		}
	};

	class fixed_frequency_stepper_controller : public stepper_controller, public i_velocity_driver
	{
		void(*interrupt_handle)(void*, int) = nullptr;
		void * interrupt_priv = nullptr;

	public:
		float speed_to_shift = 1;
		float freq = 1;
		int64_t current_shift = 0;

		fixed_frequency_stepper_controller(robo::stepper * stepper);

		void set_frequency(float freq)
		{
			this->freq = freq;
			evaluate();
		}

		void set_interrupt_handler(void(*handle)(void*, int), void * arg)
		{
			this->interrupt_handle = handle;
			this->interrupt_priv = arg;
		}

		void set_velocity(double speed) override;

		int constant_frequency_serve();

		double feedback_velocity() override 
		{
			return (float)current_shift / speed_to_shift;
		}

	private:
		void evaluate() override
		{
			speed_to_shift = freq * units_in_step// / discrete_time_frequency()
			;
		}
	};
}

#endif