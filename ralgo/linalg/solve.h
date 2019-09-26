#ifndef RALGO_LINALG_SOLVE_H
#define RALGO_LINALG_SOLVE_H

#include <ralgo/vector.h>
#include <ralgo/matrix.h>
#include <ralgo/vecops.h>
#include <ralgo/matops.h>
#include <ralgo/util/svd.h>

namespace ralgo
{
	template<class X, class A, class B>
	void solve_linear_equation_system(X& x, const A& a, const B& b)
	{
		auto m = a.size1();
		auto n = a.size2();

		assert(x.size() == n);
		assert(m == b.size());

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