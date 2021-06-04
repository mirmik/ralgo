#include <doctest/doctest.h>
#include <ralgo/linalg/matrix.h>
#include <ralgo/linalg/matops.h>
#include <ralgo/linalg/solve.h>

#include <memory>

// x = A*b

TEST_CASE("solve") 
{
	ralgo::matrix<float> A;
	std::vector<float> x;

	A = ralgo::matops::diag({1.f, 1.f, 1.f});
	x = std::vector {1.f, 1.f, 1.f};

	auto b = ralgo::linalg::solve(A, x);
}
