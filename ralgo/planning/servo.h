#ifndef RALGO_PLANNING_SERVO_H
#define RALGO_PLANNING_SERVO_H

//#include <ralgo/planning/stepctr.h>

#include <igris/sync/syslock.h>
#include <ralgo/planning/speed_driver.h>

namespace ralgo
{
	// Интерфейсный класс для реализации контроллера управления одной оси по критериям положения и скорости.
	class phase_driver
	{
		//int64_t target;
		float poskoeff = 10; // T = 1 / k

		ralgo::speed_driver * speed_driver;
	public:
		phase_driver(ralgo::speed_driver * speed_driver)
			: speed_driver(speed_driver) {}

		void set_phase(int64_t target_posunit, float posunit_per_timeunit)
		{
			system_lock();
			int64_t current = speed_driver->target_impulse_position;
			system_unlock();

			auto diff = target_posunit - current;

			float evalspeed = posunit_per_timeunit + poskoeff * diff;

			speed_driver->set_speed(evalspeed);
		}
	};
}

#endif