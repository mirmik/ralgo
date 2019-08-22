#ifndef RALGO_PLANNING_SPEED_DRIVER_H
#define RALGO_PLANNING_SPEED_DRIVER_H

#include <ralgo/planning/disctime.h>
#include <igris/util/iteration_counter.h>

namespace ralgo 
{
	// Интерфейсный класс для реализации контроллера управления одной оси по скорости.
	class phase_driver 
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
		virtual void set_speed(float posunit_per_timeunit) = 0;
		virtual void power(bool en) = 0;

		// Отношение drive_position к реально выдаваемому 
		// числу смен фаз.
		// Например: для MitsuServo это будет
		// electronic_gear / 4, т.к. на подачу одного импульса 
		// требуется четыре смены фаз.
		int gear = 1000 / 4; 

	public:
		virtual int32_t get_accum() = 0;

		phase_driver() 
		{
			poskoeff = 100 / ralgo::discrete_time_frequency();
		}

		void set_phase(int64_t tgtpos, float posunit_per_timeunit)
		{
			// Счетчик меняется в прерывании, так что 
			// снимаем локальную копию.
			auto current = target_position();

			// Ошибка по установленному значению.
			auto diff = tgtpos - current;

			// Скорость вычисляется как
			// сумма уставной скорости на 
			float evalspeed = 
				(posunit_per_timeunit + poskoeff * diff) / gear;

			//DPRINT(posunit_per_timeunit);
			//DPRINT(diff);
			/*
			DPRINT(tgtpos);
			DPRINT(current);
			DPRINT(evalspeed);
			//delay(10);
			//do_after_iteration(5) while(1);
			*/
			set_speed(evalspeed);
		}

		int64_t target_position() 
		{
			igris::syslock_guard lock();
			return control_steps_counter * gear + get_accum();
		}

		void swift_zero(int64_t imps) 
		{
			igris::syslock_guard lock();
			control_steps_counter = (target_position() - imps) / gear;
		}

		void set_gear(int g) 
		{
			gear = g;
		}
	};
}

#endif