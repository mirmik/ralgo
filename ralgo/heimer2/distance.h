/**
	Функции для работы с расстояниями.
	В библиотеке heimer расстояния хранятся в типе int64_t представлены в формате с фиксированной точкой
	Точность по умолчанию состовляет 2**32 == 4294967296 (примерно один нанометр).
*/

#ifndef RALGO_HEIMER_DISTANCE_H
#define RALGO_HEIMER_DISTANCE_H

#include <igris/compiler.h>
#include <stdint.h>

#define DISTANCE_SHIFT (24)

#define DISTANCE_MULTIPLIER (1LL << DISTANCE_SHIFT)

__BEGIN_DECLS

static inline
double distance_fixed_to_float(int64_t arg) 
{
	return (double)arg / (double)(DISTANCE_MULTIPLIER);
}

static inline
int64_t distance_float_to_fixed(double real) 
{
	return (int64_t)(real * DISTANCE_MULTIPLIER);
}

__END_DECLS

#endif