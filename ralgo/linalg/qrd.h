#ifndef RALGO_LINALG_QRD_H
#define RALGO_LINALG_QRD_H

// QR decomposition

#include <ralgo/linalg/matrix.h>
#include <ralgo/linalg/matops.h>

namespace ralgo
{
	template <class M, class TQ, class TR>
	class QRD
	{
	public:
		M a;
		TQ q;
		TR r;

		int status;

	public:
		using value_type = typename M::value_type;

		QRD(const M & _mat) : a(_mat)
		{
			if (_mat.rows() != _mat.rows())
			{
				status = -1;
				return;
			}

			int n = a.rows();
			int m = a.rows();

			q.resize(n, m);
			r.resize(n, m);

			ralgo::matops::clean(q);
			ralgo::matops::clean(r);

			ralgo::matops::assign(a, q);

			decompose();
		}

		void decompose()
		{
			int i, j;
			double anorm, tol = 10e-7;

			int m = a.rows();
			int n = a.rows();

			for (i = 0; i < n; i++)
			{
				r.at(i,i) = ralgo::vecops::length(a.row(i)); // r_ii = ||a_i||

				if (r.at(i,i) > tol)
				{
					// a_i = a_i/r_ii
					ralgo::vecops::scalar_div(a.row(i), r.at(i,i), a.row(i)); 
				}
				else if (i == 0)  // set a[0] = [1 0 0 ... 0]^T
				{
					a.at(i,0) = 1;
					for (j = 1; j < m; j++)
					{
						a.at(i,j) = 0;
					}
				}
				else  // need to choose a_i orthogonal to < a_1, ... a_{i-1} >
				{
					for (j = 0; j < m; j++)
					{
						a.at(i,j) = -a.at(0,i) * a.at(0,j);
					}
					a.at(i,i) += 1;

					for (j = 1; j < i; j++)
					{
						ralgo::vecops::scalar_sub(a.row(j), a.at(j,i), a.row(i));
					}

					anorm = ralgo::vecops::length(a.row(i));
					ralgo::vecops::scalar_div(a.row(i), anorm, a.row(i));
				}

				for (j = i + 1; j < n; j++)
				{
					r.at(j,i) = ralgo::vecops::dot_product(a.row(i), a.row(j)); // r_ij = a_i*a_j
					ralgo::vecops::scalar_sub(a.row(i), r.at(j,i), a.row(j)); // a_j -= r_ij a_i
				}
			}

			print();
		}

		void print()
		{
			nos::println("Q:"); nos::print_matrix(q);
			nos::println("R:"); nos::print_matrix(r);
			nos::println();

			nos::println("QR:"); nos::print_matrix(ralgo::matops::multiply(q, r));

		}
	};

	template <
	    class M,
	    class Q = ralgo::matrix<typename M::value_type, ralgo::row_order<typename M::value_type>, std::allocator<typename M::value_type>>,
	    class R = ralgo::matrix<typename M::value_type, ralgo::row_order<typename M::value_type>, std::allocator<typename M::value_type>>
	    >
	QRD<M, Q, R> qrd(const M & mat)
	{
		return QRD <
		       M,
		       Q,
		       R
		       > (mat);
	}
}

#endif