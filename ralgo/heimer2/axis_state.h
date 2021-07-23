#ifndef RALGO_HEIMER_AXIS_STATE
#define RALGO_HEIMER_AXIS_STATE

#include <stdint.h>

#include <igris/compiler.h>
#include <ralgo/heimer2/heimer_types.h>
#include <ralgo/heimer2/signal.h>

struct signal_processor;
struct axis_state 
{
	struct signal_head sig;

	position_t ctrpos; /// Контрольная позиция в формате с фиксированной точкой	
	velocity_t ctrvel; /// Сигнал управления по скорости.

	position_t feedpos; /// Данные нижнего уровня о позиции
	velocity_t feedvel; /// Данные нижнего уровня о скорости
};

__BEGIN_DECLS

void axis_state_init(struct axis_state * state, const char * name);

int axis_state_info(struct signal_head * sig, char * data, int maxsize);

struct axis_state * create_axis_state(const char * name);

__END_DECLS

#endif