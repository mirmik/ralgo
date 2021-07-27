#include <doctest/doctest.h>
#include <ralgo/lp/point_in_hexagon.h>

TEST_CASE("point_in_hexagon__copy_rows_by_indexes_transposed") 
{
	double A[] = 
	{
		1, 2,
		5, 6,
		8, 9,
		11, 12,
		14, 15,
	};

	double target[6]; 
	int idxarr[] = {1, 3, 4};

	point_in_hexagon__copy_rows_by_indexes_transposed(A, 2, 5, idxarr, 3, target);	

	CHECK_EQ(target[0], 5);
	CHECK_EQ(target[1], 11);
	CHECK_EQ(target[2], 14);

	CHECK_EQ(target[3], 6);
	CHECK_EQ(target[4], 12);
	CHECK_EQ(target[5], 15);
}

TEST_CASE("point_in_simplex") 
{
	double A[] = 
	{
		0, 0,
		2, 0,
		0, 2
	};

	double target0[] = { 0.5, 0.5 };
	CHECK_EQ(point_in_simplex_d(A, 2, target0), 1);

	double target1[] = { -0.1, 0.5 };
	CHECK_EQ(point_in_simplex_d(A, 2, target1), 0);

	double target2[] = { 0.5, 2.1 };
	CHECK_EQ(point_in_simplex_d(A, 2, target2), 0);

	double target3[] = { 1, 1 };
	CHECK_EQ(point_in_simplex_d(A, 2, target3), 1);

	double target4[] = { 0, 0 };
	CHECK_EQ(point_in_simplex_d(A, 2, target4), 1);
}

/*TEST_CASE("point_in_hexagon") 
{
	double A[] = 
	{
		0, 0,
		2, 0,
		0, 2,
		2, 2
	};

	double target0[] = { 1, 1 };
	CHECK_EQ(point_in_hexagon_d(A, 2, 4, target0), 1);

	double target1[] = { 3, 1 };
	CHECK_EQ(point_in_hexagon_d(A, 2, 4, target1), 0);

	double target2[] = { 0.5, 1 };
	CHECK_EQ(point_in_hexagon_d(A, 2, 4, target2), 1);

	double target3[] = { 1.5, 1.3 };
	CHECK_EQ(point_in_hexagon_d(A, 2, 4, target3), 1);

	double target4[] = { -0.1, 1 };
	CHECK_EQ(point_in_hexagon_d(A, 2, 4, target4), 0);

	double target5[] = { 1, 2.1 };
	CHECK_EQ(point_in_hexagon_d(A, 2, 4, target5), 0);
} */
