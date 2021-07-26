#ifndef RALGO_CLINALG_VECOPS_H
#define RALGO_CLINALG_VECOPS_H

#include <igris/compiler.h>
#include <math.h>

__BEGIN_DECLS

static inline
double vecops_point_distance_d(double * a, double * b, int dim) 
{
	double acc = 0;

	for (int i = 0; i < dim; ++i) 
	{
		double diff = a[i] - b[i];
		acc += diff * diff;
	}

	return sqrt(acc);
}

__END_DECLS

#endif