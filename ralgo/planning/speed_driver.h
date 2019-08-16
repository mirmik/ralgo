#ifndef RALGO_PLANNING_SPEED_DRIVER_H
#define RALGO_PLANNING_SPEED_DRIVER_H

namespace ralgo 
{
	// Интерфейсный класс для реализации контроллера управления одной оси по скорости.
	class speed_driver 
	{
	public:
		volatile int64_t target_impulse_position = 0;

	public:
		virtual void set_speed(float posunit_per_timeunit) = 0;
	};
}

#endif