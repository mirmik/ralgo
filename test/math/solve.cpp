#include <doctest/doctest.h>
#include <ralgo/linalg/matrix.h>
#include <ralgo/linalg/matops.h>
#include <ralgo/linalg/solve.h>

#include <memory>

// x = A*b

TEST_CASE("solve") 
{
	ralgo::matrix<float> A;
	std::vector<float> x, b;

	A = ralgo::matops::diag({1.f, 1.f, 1.f});
	x = std::vector {1.f, 1.f, 1.f};
	b = ralgo::linalg::solve(A, x);
	CHECK_EQ(b[0],1);
	CHECK_EQ(b[1],1);
	CHECK_EQ(b[2],1);

	A = ralgo::matops::diag({2.f, 1.f, 2.f});
	x = std::vector {1.f, 1.f, 1.f};
	b = ralgo::linalg::solve(A, x);
	CHECK_EQ(b[0],0.5);
	CHECK_EQ(b[1],1);
	CHECK_EQ(b[2],0.5);
}
