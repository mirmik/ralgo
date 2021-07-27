#include <ralgo/lp/point_in_hexagon.h>
#include <ralgo/util/index_brute_force.h>

#include <ralgo/clinalg/solve.h>
#include <igris/dprint.h>

void point_in_hexagon__copy_rows_by_indexes_transposed(
    double * A,
    int dim,
    int points,
    int * indexes,
    int indexes_total,
    double * result_matrix)
{
	for (int ii = 0; ii < indexes_total; ++ii)
	{
		int index = indexes[ii];

		for (int j = 0; j < dim; ++j)
		{
			*(result_matrix + j * (indexes_total) + ii) = *(A + index * dim + j);
		}
	}
}

/**
	Решает задачу о нахождении точки внутри симлекса - выпуклой оболочке n+1 точки n мерного пространства.
*/
int point_in_simplex_d(
    double * A,
    int dim,
    double * target
)
{
	double matrix [(dim + 1) * (dim + 1)];
	double exttgt[dim + 1];
	double result[dim + 1];

	memset(result, 0, sizeof(result))

	for (int pnt = 0; pnt < dim + 1; ++pnt)
	{
		for (int i = 0; i < dim; ++i)
		{
			*(matrix + i * (dim + 1) + pnt) = *(A + pnt * dim + i);
		}
	}

	for (int i = 0; i < dim + 1; ++i)
	{
		*(matrix + dim * (dim + 1) + i) = 1;
	}

	for (int i = 0; i < dim; ++i)
	{
		*(exttgt + i) = target[i];
	}
	exttgt[dim] = 1;

	for ( int i = 0; i < dim + 1; i++ )
	{
		for ( int j = 0; j < dim + 1; j++ )
		{
			dpr(*(matrix + i * (dim + 1) + j));
			dpr(" ");
		}
		dln();
	}

	for ( int i = 0; i < dim + 1; i++ )
	{
		dpr(*(exttgt + i));
		dpr(" ");
	}
	dln();

	linalg_square_solve_d(A, dim+1, exttgt, result);

	for ( int i = 0; i < dim + 1; i++ )
	{
		dpr(*(result + i));
		dpr(" ");
	}
	dln();

	for (int i = 0; i < dim + 1; ++i)
	{
		if (result[i] > 1 || result[i] < 0)
			return 0;
	}

	return 1;
}


int point_in_hexagon_d(
    double * A,
    int dim,
    int points,
    double * target
)
{
	double matrix[(dim + 1) * (dim + 1)];
	double xresult[dim + 1];
	int base_coords[dim + 1];

	index_brute_force_init(base_coords, dim + 1);

	// Устанавливаем нижнюю строку матрицы базовых решений.
	for (int i = 0; i < dim + 1; ++i)
	{
		*(matrix + (dim + 1) * (dim) + i) = 1;
	}

	do
	{
		point_in_hexagon__copy_rows_by_indexes_transposed(A, dim, points, base_coords, dim + 1, matrix);

		//if (sts)
		//	continue;

		//for ()
	}
	while (index_brute_force_next(base_coords, dim + 1, points) == 0);

	return 0;
}
