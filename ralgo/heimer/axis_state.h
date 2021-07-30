/** @file */ 
#ifndef RALGO_HEIMER_AXIS_STATE
#define RALGO_HEIMER_AXIS_STATE

#include <stdint.h>

#include <igris/compiler.h>
#include <ralgo/heimer/heimer_types.h>
#include <ralgo/heimer/signal.h>

namespace heimer
{

	/**
		Внимание: Данные не в единицах системы си.

		Данные по положению даются в условных едицах.
		Данные по скорости в условных единицах делённых на дискрет времени системы.
	*/

	class axis_state : public signal_head
	{
	public:
		position_t ctrpos;
		velocity_t ctrvel;
		position_t feedpos;
		velocity_t feedvel;

	public:
		axis_state() = default;
		axis_state(const char * name);

		void init(const char * name);
		int info(char * data, int maxsize) override;
	};
}

#endif