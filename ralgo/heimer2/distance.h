/**
	Функции для работы с расстояниями.
	В библиотеке heimer расстояния хранятся в типе int64_t представлены в формате с фиксированной точкой
	Точность по умолчанию состовляет 2**32 == 4294967296 (примерно один нанометр).
*/

#ifndef RALGO_HEIMER_DISTANCE_H
#define RALGO_HEIMER_DISTANCE_H

#include <ralgo/heimer2/heimer_types.h>
#include <igris/compiler.h>
#include <igris/math.h>
#include <stddef.h>
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

static inline 
double heimdist_restore(position_t dist) 
{
	return distance_fixed_to_float(dist);
}

static inline 
velocity_t heimvel(double vel) 
{
	return vel * DISTANCE_MULTIPLIER;
}

static inline 
double heimvel_restore(velocity_t vel) 
{
	return vel / DISTANCE_MULTIPLIER;
}

static inline 
acceleration_t heimacc(double acc) 
{
	return acc * DISTANCE_MULTIPLIER;
}

static inline 
position_t heimdeg(double dist) 
{
	return distance_float_to_fixed(dist / 180. * M_PI);
}

static inline 
float heimpos_cos(position_t x) 
{
	return cosf(distance_fixed_to_float(x));
} 

static inline 
float heimpos_sin(position_t x) 
{
	return sinf(distance_fixed_to_float(x));
} 

__END_DECLS

#endif