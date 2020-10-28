#ifndef RALGO_LINALG_SOLVE_H
#define RALGO_LINALG_SOLVE_H

#include <ralgo/linalg/vector_view.h>
#include <ralgo/linalg/matrix_view.h>
#include <ralgo/linalg/vecops.h>
#include <ralgo/linalg/matops.h>
#include <ralgo/linalg/svd.h>

namespace ralgo
{
	template<class X, class A, class B>
	void solve_linear_equation_system(X& x, const A& a, const B& b)
	{
		auto m = a.size1();
		auto n = a.size2();

		assert(std::size(x) == n);
		assert(m == std::size(b));

		double u[m * n];
		double v[n * n];
		double w[n];

		ralgo::matrix_view<double> U(u, m, n);
		ralgo::matrix_view<double> V(v, n, n);
		ralgo::vector_view<double> W(w, n);

		auto svd = ralgo::make_SVD(a, U, V, W);
		svd.solve(b, x);
	}
}

#endif