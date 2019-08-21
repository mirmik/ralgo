#ifndef RALGO_PLANNING_SPEED_DRIVER_H
#define RALGO_PLANNING_SPEED_DRIVER_H

namespace ralgo 
{
	// Интерфейсный класс для реализации контроллера управления одной оси по скорости.
	class phase_driver 
	{
		float poskoeff = 10; // T = 1 / k

	public:
		volatile int64_t target_impulse_position = 0;

	public:
		virtual void set_speed(float posunit_per_timeunit) = 0;
		virtual void power(bool en) = 0;

		int gear = 4;

	public:
		void set_phase(int64_t tgtpos, float posunit_per_timeunit)
		{
			system_lock();
			int64_t current = target_impulse_position;
			system_unlock();

			auto diff = tgtpos * gear - current;

			float evalspeed = 
				posunit_per_timeunit * gear 
				+ poskoeff * diff;

			set_speed(evalspeed);
		}

		void swift_zero(int64_t imps) 
		{
			target_impulse_position -= imps * gear;
		}
	};
}

#endif