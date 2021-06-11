#ifndef RALGO_LINALG_TRIANGLE_SOLVE_H
#define RALGO_LINALG_TRIANGLE_SOLVE_H

#include <ralgo/util/helpers.h>
#include <ralgo/linalg/proxy.h>

namespace ralgo
{
	template<class X, class M, class B>
	void unary_solve(const M & a, const B & b, X&& x)
	{
		ralgo::resize(x, ralgo::size(b));
		
		ralgo::transposed_matrix_proxy a_transposed(a);
		ralgo::matops::multiply(a_transposed, b, x);
	}

	template<class X = void, class M, class B>
	defvec_t<X, B> unary_solve(M && a, B && b)
	{
		defvec_t<X, B> x;
		unary_solve(a, b, x);
		return x;
	}


	// B = MX -> X
	template<class X, class M, class B>
	void pivot_solve(const M & a, const B & b, X&& x)
	{
		ralgo::resize(x, ralgo::size(b));

		typename M::value_type acc;
		for (int i = 0; i < a.rows(); ++i)
		{
			for (int j = 0; j < a.cols(); ++j)
			{
				auto val = ralgo::get(a, i, j);
				if (val) ralgo::get(x, j) = ralgo::get(b, i);
			}
		}
	}

	template<class X = void, class M, class B>
	defvec_t<X, B> pivot_solve(M && a, B && b)
	{
		defvec_t<X, B> x;
		pivot_solve(a, b, x);
		return x;
	}


	// B = MX -> X
	template<class X, class M, class B>
	void L_triangle_solve(const M & a, const B & b, X&& x)
	{
		ralgo::resize(x, ralgo::size(b));

		typename M::value_type acc;
		for (int i = 0; i < a.rows(); ++i)
		{
			acc = ralgo::get(b, i);

			for (int j = 0; j < i; j++)
			{
				acc -= x[j] * ralgo::get(a, i, j);
			}

			x[i] = acc / ralgo::get(a, i, i);
		}
	}

	template<class X = void, class M, class B>
	defvec_t<X, B> L_triangle_solve(M && a, B && b)
	{
		defvec_t<X, B> x;
		L_triangle_solve(a, b, x);
		return x;
	}


	// B = MX -> X
	template<class X, class M, class B>
	void U_triangle_solve(const M & a, const B & b, X&& x)
	{
		ralgo::resize(x, ralgo::size(b));

		typename M::value_type acc;
		for (int i = a.rows() - 1; i >= 0; --i)
		{
			acc = ralgo::get(b, i);

			for (int j = i + 1; j < a.cols(); j++)
			{
				acc -= x[j] * ralgo::get(a, i, j);
			}

			x[i] = acc / ralgo::get(a, i, i);
		}
	}

	template<class X = void, class M, class B>
	defvec_t<X, B> U_triangle_solve(M && a, B && b)
	{
		defvec_t<X, B> x;
		U_triangle_solve(a, b, x);
		return x;
	}
}

#endif