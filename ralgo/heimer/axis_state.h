#ifndef RALGO_HEIMER_AXIS_STATE
#define RALGO_HEIMER_AXIS_STATE

#include <stdint.h>

#include <igris/compiler.h>
#include <ralgo/heimer/heimer_types.h>
#include <ralgo/heimer/signal.h>

namespace heimer
{
	class axis_state : public signal_head
	{
	public:
		position_t ctrpos; /// Контрольная позиция в формате с фиксированной точкой
		velocity_t ctrvel; /// Сигнал управления по скорости.

		position_t feedpos; /// Данные нижнего уровня о позиции
		velocity_t feedvel; /// Данные нижнего уровня о скорости

	public:
		axis_state() = default;
		axis_state(const char * name);

		void init(const char * name);
		int info(char * data, int maxsize) override;
	};
}

#endif