#ifndef RALGO_LINALG_LU_DECOMPOSITION_H
#define RALGO_LINALG_LU_DECOMPOSITION_H

/**
	@file
*/

#include <ralgo/linalg/matrix.h>
#include <ralgo/linalg/matops.h>
#include <ralgo/linalg/trivial_solve.h>

#include <nos/print.h>

namespace ralgo
{
	template <class M, class TP, class TL, class TU>
	class PLUD
	{
	public:
		M a;
		TP p;
		TL l;
		TU u;

		int status;

		PLUD(const M & _mat)
			: PLUD(_mat, TP{}, TL{}, TU{})
		{}

		PLUD(const M & _mat, const TP & _p, const TL & _l, const TU & _u)
			: a(_mat), p(_p), l(_l), u(_u)
		{
			if (_mat.rows() != _mat.cols())
			{
				status = -1;
				return;
			}

			int n = a.rows();

			p.resize(n, n);
			l.resize(n, n);
			u.resize(n, n);

			ralgo::matops::clean(p);
			ralgo::matops::clean(l);
			ralgo::matops::clean(u);

			ralgo::matops::eye(p);
			ralgo::matops::eye(l);
			ralgo::matops::assign(a, u);

			decompose();
		}

		void decompose()
		{
			const int n = a.rows();

			for ( int i = 0; i < n; i++ )
			{
				double pivotValue = 0;
				int pivot = -1;
				for ( int row = i; row < n; row++ )
				{
					if ( std::abs(u(row, i)) > pivotValue )
					{
						pivotValue = std::abs(u[ row ][ i ]);
						pivot = row;
					}
				}
				if ( pivotValue != 0 )
				{
					ralgo::vecops::swap(p.row(pivot), p.row(i));
					ralgo::vecops::swap(u.row(pivot), u.row(i));
					for ( int j = i + 1; j < n; j++ )
					{
						u(j, i) /= u(i, i);
						for ( int k = i + 1; k < n; k++ )
							u(j, k) -= u(j, i) * u(i, k);
					}
				}
			}

			for ( int i = 0; i < n; i++ )
			{
				for ( int j = 0; j < i; j++ )
				{
					l(i, j) = u(i, j);
					u(i, j) = 0;
				}
			}

			ralgo::matops::square_inline_transpose(p);
		}

		template <class X, class B>
		void solve(const B& b, X&& x)
		{
			vector_value_t<B> ybuf[b.size()];
			vector_view y(ybuf, b.size());

			x.resize(b.size());

			// Последовательно применяем решения матриц простого вида.
			// Данная процедура эквивалентна последовательному
			// умножению на обращенные P, L, U.
			pivot_solve(p, b, x);
			L_triangle_solve(l, x, y);
			U_triangle_solve(u, y, x);
		}

		template <class X = void, class B>
		defvec_t<X, B> solve(B && b)
		{
			defvec_t<X, B> x;
			solve(b, x);
			return x;
		}
	};

	template <
	    class M,
	    class P = ralgo::matrix<typename M::value_type, ralgo::row_order<typename M::value_type>, std::allocator<typename M::value_type>>,
	    class L = ralgo::matrix<typename M::value_type, ralgo::row_order<typename M::value_type>, std::allocator<typename M::value_type>>,
	    class U = ralgo::matrix<typename M::value_type, ralgo::row_order<typename M::value_type>, std::allocator<typename M::value_type>>
	    >
	PLUD<M, P, L, U> plud(const M & mat)
	{
		return PLUD <
		       M,
		       P,
		       L,
		       U
		       > (mat);
	}

	template <
	    class M,
	    class P = ralgo::matrix<typename M::value_type, ralgo::row_order<typename M::value_type>, std::allocator<typename M::value_type>>,
	    class L = ralgo::matrix<typename M::value_type, ralgo::row_order<typename M::value_type>, std::allocator<typename M::value_type>>,
	    class U = ralgo::matrix<typename M::value_type, ralgo::row_order<typename M::value_type>, std::allocator<typename M::value_type>>
	    >
	PLUD<M, P, L, U> plud(const M & mat, const P & p, const L & l, const U & u)
	{
		return PLUD <
		       M,
		       P,
		       L,
		       U
		       > (mat, p, l, u);
	}
}

#endif