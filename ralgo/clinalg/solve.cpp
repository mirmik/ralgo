#include <ralgo/clinalg/solve.h>
#include <ralgo/linalg/plud.h>

void linalg_square_solve_d(double * A, int n, double * b, double * x) 
{
	double P[n*n];
	double L[n*n];
	double U[n*n];

	ralgo::matrix_view<double> A_view(A, n, n);
	ralgo::matrix_view<double> P_view(P, n, n);
	ralgo::matrix_view<double> L_view(L, n, n);
	ralgo::matrix_view<double> U_view(U, n, n);

	ralgo::vector_view<double> b_view(b, n);
	ralgo::vector_view<double> x_view(x, n);

	auto plud_decomposition = ralgo::plud(A_view, P_view, L_view, U_view);
	plud_decomposition.solve(b_view, x_view);
}