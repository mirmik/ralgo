#ifndef RALGO_HEIMER_AXIS_STATE
#define RALGO_HEIMER_AXIS_STATE

#include <stdint.h>

#include <igris/compiler.h>
#include <ralgo/heimer2/signal.h>

struct axis_state 
{
	struct signal_head sig;

	int64_t ctrpos; /// Контрольная позиция в формате с фиксированной точкой	
	float   ctrvel; /// Сигнал управления по скорости.

	int64_t feedpos; /// Данные нижнего уровня о позиции
	float   feedspd; /// Данные нижнего уровня о скорости
};

__BEGIN_DECLS

void axis_state_init(struct axis_state * state, const char * name);

struct axis_state * create_axis_state(const char * name);

__END_DECLS

#endif