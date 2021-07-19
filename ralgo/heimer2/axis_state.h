#ifndef RALGO_HEIMER_AXIS_STATE
#define RALGO_HEIMER_AXIS_STATE

#include <stdint.h>

struct axis_state 
{
	int64_t ctrpos; /// Контрольная позиция в формате с фиксированной точкой	
	float   ctrvel; /// Сигнал управления по скорости.

	int64_t feedpos; /// Данные нижнего уровня о позиции
	float   feedspd; /// Данные нижнего уровня о скорости
};

#endif