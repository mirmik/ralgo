#ifndef RALGO_PLANNING_SPEED_DRIVER_H
#define RALGO_PLANNING_SPEED_DRIVER_H

#include <ralgo/planning/disctime.h>
#include <igris/util/iteration_counter.h>

#include <nos/trace.h>

namespace ralgo
{
	class phase_driver
	{
		float poskoeff;

	public:
		phase_driver()
		{
			set_timeconst(0.01);
		}

		// Установить постоянную времени компенсации ошибки позиционирования.
		void set_timeconst(float T)
		{
			poskoeff = 1 / (T * ralgo::discrete_time_frequency());
		}

		// Задать фазовое управление в расчетных единицах.
		// Реализация может использовать множители, но расчеты следует
		// проводить в расчетных единицах.
		//void set_phase(float pos, float speed);
		void set_phase(float tgtpos, float tgtspd)
		{
			// Счетчик меняется в прерывании, так что
			// снимаем локальную копию.
			auto current = current_position();

			// Ошибка по установленному значению.
			auto diff = tgtpos - current;

			// Скорость вычисляется как
			// сумма уставной скорости на
			auto evalspeed = tgtspd  + poskoeff * diff;

			set_speed(evalspeed);
		}
		// Задать скорость в расчетных единицах.
		virtual void set_speed(float speed) = 0;

		// Получить позицию в единицах вычисления.
		virtual float current_position() = 0;
		virtual float current_speed() = 0;

		// Опциональная операция, устанавливающая текущее расчетное положение,
		// драйвера. Поведение зависит от имплементации.
		virtual void set_current_position(float pos)
		{ BUG(); }

		// Активировать драйвер / включить питание.
		// Проверять включенность при вызове serve.
		virtual void enable_power(bool en) = 0;
	};


	// Интерфейсный класс для реализации контроллера управления одной оси по скорости.
	/*class phase_driver
	{
		// TODO: phase_driver фактически слился со stepctr.
		// их надо или объединить или phase_driver следует
		// сделать интерфейсом.
		// Рекомендуется принять это решение после встречи
		// с любым принципиально отличным типом фазового драйвера.

		// Задаёт постоянную времени коррекции уставного положения
		// (Инверсно). Масштаб времени совпадает
		// T = 1 / k
		float poskoeff = 10;

	public:
		volatile int64_t control_steps_counter = 0;

	public:
		virtual void set_phases_speed(float phases_per_timeunit) = 0;
		virtual float phases_speed() = 0;
		virtual void power(bool en) = 0;

		// Отношение drive_position к реально выдаваемому
		// числу смен фаз.
		// Например: для MitsuServo это будет
		// electronic_gear / 4, т.к. на подачу одного импульса
		// требуется четыре смены фаз.
		int gear = 1000 / 4;
		float unit_gain = 1;

	public:
		virtual float get_accum_part() = 0;
		virtual void set_accum_part(float) = 0;

		phase_driver() : control_steps_counter(0)
		{
			poskoeff = 100 / ralgo::discrete_time_frequency();
		}

		void set_phase(int64_t tgtpos, float posunit_per_timeunit)
		{
			// Счетчик меняется в прерывании, так что
			// снимаем локальную копию.
			auto current = control_position();

			// Ошибка по установленному значению.
			auto diff = tgtpos - current;

			// Скорость вычисляется как
			// сумма уставной скорости на
			float evalspeed =
				(posunit_per_timeunit + poskoeff * diff) / gear;

			set_phases_speed(evalspeed);
		}

		void set_speed(float drive_pulses_per_timeunit)
		{
			//PRINT(drive_pulses_per_timeunit);
			set_phases_speed(drive_pulses_per_timeunit / gear);
		}

		float speed()
		{
			return phases_speed() * gear;
		}

		int64_t control_position()
		{
			igris::syslock_guard lock();

			//PRINT(get_accum());
			//PRINT((int32_t)control_steps_counter);
			//PRINT(gear);

			return control_steps_counter * gear + get_accum_part() * gear;
		}

		void set_control_position(int64_t pos)
		{
			igris::syslock_guard lock();

			int64_t counter = pos / gear;
			int64_t accum = pos % gear;

			//PRINT(counter);
			//PRINT(accum);

			control_steps_counter = counter;
			set_accum_part((float)accum / gear);
		}

		float read_coord(float multiplier)
		{
			//PRINT(multiplier);
			return control_position() / multiplier;
		}

		void swift_zero(int64_t imps)
		{
			igris::syslock_guard lock();
			control_steps_counter = (control_position() - imps) / gear;
		}

		void set_gear(int g)
		{
			gear = g;
		}

		void set_unit_gain(float m)
		{
			unit_gain = m;
		}

		float control_position_unit()
		{
			return control_position() / unit_gain;
		}
	};*/
}

#endif