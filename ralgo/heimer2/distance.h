/**
	Функции для работы с расстояниями.
	В библиотеке heimer расстояния хранятся в типе int64_t представлены в формате с фиксированной точкой
	Точность по умолчанию состовляет 2**32 == 4294967296 (примерно один нанометр).
*/

#ifndef RALGO_HEIMER_DISTANCE_H
#define RALGO_HEIMER_DISTANCE_H

#include <ralgo/heimer2/heimer_types.h>
#include <igris/compiler.h>
#include <stdint.h>

#define DISTANCE_SHIFT (24)

#define DISTANCE_MULTIPLIER (1LL << DISTANCE_SHIFT)

__BEGIN_DECLS

static inline
double distance_fixed_to_float(position_t arg) 
{
	return (double)arg / (double)(DISTANCE_MULTIPLIER);
}

static inline
position_t distance_float_to_fixed(double real) 
{
	return (position_t)(real * DISTANCE_MULTIPLIER);
}

static inline 
position_t heimdist(double dist) 
{
	return distance_float_to_fixed(dist);
} 

__END_DECLS

#endif