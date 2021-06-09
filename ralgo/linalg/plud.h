#ifndef RALGO_LINALG_LU_DECOMPOSITION_H
#define RALGO_LINALG_LU_DECOMPOSITION_H

/**
	@file
*/

#include <ralgo/linalg/matrix.h>
#include <ralgo/linalg/matops.h>

namespace ralgo
{
	template <class M, class TL, class TU, class TP>
	class PLUD
	{
	public:
		M a;
		TP p;
		TL l;
		TU u;

		int status;

		PLUD(const M & _mat) : a(_mat)
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
		}

		void print()
		{
			nos::println("A:"); nos::print_matrix(a);
			nos::println("P:"); nos::print_matrix(p);
			nos::println("L:"); nos::print_matrix(l);
			nos::println("U:"); nos::print_matrix(u);
			nos::println();
		}
	};

	template <
	    class M,
	    class L = ralgo::matrix<typename M::value_type, ralgo::row_order<typename M::value_type>, std::allocator<typename M::value_type>>,
	    class U = ralgo::matrix<typename M::value_type, ralgo::row_order<typename M::value_type>, std::allocator<typename M::value_type>>,
	    class P = ralgo::matrix<typename M::value_type, ralgo::row_order<typename M::value_type>, std::allocator<typename M::value_type>>
	    >
	PLUD<M, L, U, P> plud(const M & mat)
	{
		return PLUD <
		       M,
		       L,
		       U,
		       P
		       > (mat);
	}
}

#endif