#include <ralgo/linalg/backpack.h>
#include <main.h>

using namespace ralgo;

#define EPSILON 1e-6

LT_BEGIN_TEST(ralgo_test_suite, backpack)
{
	double arr[12];
	matrix_view<double> mat(arr,4,3);
	matops::copy_from_cols(mat, dvec2{ {1,0,0,1}, {0,1,0,-1}, {0,0,1,0} });

	dvec res{0,0,0};
	dvec tgt{1,1,1,0};

	svd_backpack(res, tgt, mat);

	CHECK(abs(res[0] - 1) < EPSILON);
	CHECK(abs(res[1] - 1) < EPSILON);
	CHECK(abs(res[2] - 1) < EPSILON);
}
LT_END_TEST(backpack)