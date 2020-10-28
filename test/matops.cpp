#include <ralgo/linalg/matops.h>
#include <ralgo/linalg/matrix_view.h>

#include <vector>
#include <main.h>

using namespace ralgo;

LT_BEGIN_TEST(ralgo_test_suite, matops_test)
{
	double a[9];
	double b[9];

	ralgo::matrix_view<double> amat(a, 3, 3);
	ralgo::matrix_view<double> bmat(b, 3, 3);

	CHECK_EQ(amat.size1(), 3);
	CHECK_EQ(amat.size2(), 3);

	matops::fill(amat, 22);

	CHECK_EQ(amat(0,0), 22);
	CHECK_EQ(amat(2,2), 22);
	CHECK_EQ(amat(2,1), 22);

	std::vector<double> v1[] = { {1,2,3}, {4,5,6}, {7,8,9} };
	matops::copy_from_rows(bmat, std::begin(v1), std::end(v1));

	CHECK_EQ(bmat(0,0), 1);
	CHECK_EQ(bmat(1,1), 5);
	CHECK_EQ(bmat(1,0), 4);

	std::vector<double> v2[] = { {1,2,3}, {4,5,6}, {7,8,9} };
	matops::copy_from_cols(bmat, std::begin(v2), std::end(v2));

	CHECK_EQ(bmat(0,0), 1);
	CHECK_EQ(bmat(1,1), 5);
	CHECK_EQ(bmat(0,1), 4);

}
LT_END_TEST(matops_test)