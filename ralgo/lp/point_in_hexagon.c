#include <ralgo/lp/point_in_hexagon.h>


int point_in_hexagon_d(
	double * A,
	int dim,
	int points,
	double * target 
) 
{
	double matrix[(dim + 1) * points];
	int base_coords[dim + 1];

	for (int i = 0; i < dim + 1; ++i) 
		base_coords[i] = i;
}