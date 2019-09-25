#include <ralgo/matrix.h>
#include <ralgo/util/svd.h>
#include <ralgo/vecops.h>

#include <nos/print.h>
#include <vector>

#include <malgo/nrecipes/svd.h>
#include <ralgo/util/backpack.h>
#include <ralgo/linalg/solve.h>

int main()
{	
	double a[3*3] = { 1,1,1, 1,8,3, 1,2,3 };
	ralgo::matrix_view A(a, 3, 3);

	double b[3] = {1,1,1};
	ralgo::vector_view<double> B(b, 3);

	double x[3];
	ralgo::vector_view<double> X(x, 3);

	ralgo::solve_linear_equation_system(X, A, B);

	nos::print_list(X);
}