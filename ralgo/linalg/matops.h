#ifndef RALGO_MATOPS_H
#define RALGO_MATOPS_H

#include "assert.h"

#include <utility>
#include <iterator>

#include <ralgo/linalg/vecops_base.h>
#include <ralgo/linalg/vecops.h>

#include <nos/print.h>

// Алгоритмы для матриц имеющих интерфейс схожий с интерфейсом boost.matrix
namespace ralgo
{
	namespace matops
	{
		template<class M> void fill(M& arr, const value_t<M>& val) { vecops::fill(arr, val); }

		template<class A, class B>
		void copy(A& a, const B& b)
		{
			size_t m = a.size1();
			size_t n = a.size2();

			assert(a.size1() == b.size1());
			assert(a.size2() == b.size2());

			for (unsigned int i = 0; i < m; ++i)
				for (unsigned int j = 0; j < n; ++j)
				{
					a(i, j) = b(i, j);
				}
		}

		template <class M, class VI, class VEI>
		void copy_from_rows(M& tgt, VI srcit, VEI eit)
		{
			size_t n = std::size(*srcit);
			size_t m = std::distance(srcit, eit);

			for (unsigned int i = 0; i < m; ++i)
			{
				auto& v = *srcit;
				for (unsigned int j = 0; j < n; ++j)
				{
					tgt(i, j) = v[j];
				}
				srcit++;
			}
		}
		template <class M, class VV> void copy_from_rows(M& tgt, const VV& vecs)
		{ copy_from_rows(tgt, std::cbegin(vecs), std::cend(vecs)); }

		template <class M, class VI, class VEI>
		void copy_from_cols(M& tgt, VI srcit, VEI eit)
		{
			size_t m = std::size(*srcit);
			size_t n = std::distance(srcit, eit);

			for (unsigned int j = 0; j < n; ++j)
			{
				auto& v = *srcit;
				for (unsigned int i = 0; i < m; ++i)
				{
					tgt(i, j) = v[i];
				}
				srcit++;
			}
		}
		template <class M, class VV> void copy_from_cols(M& tgt, const VV& vecs)
		{ copy_from_cols(tgt, std::cbegin(vecs), std::cend(vecs)); }


		template<class A, class B>
		void assign(const A& a, B& b)
		{
			size_t m = a.rows();
			size_t n = a.cols();
			
			b.resize(m,n);
			
			for (unsigned int i = 0; i < m; ++i)
				for (unsigned int j = 0; j < n; ++j)
				{
					b(i, j) = a(i, j);
				}
		}

		template <class A, class B, class C>
		void multiply(const A& a, const B& b, C& c)
		{
			// m must be equal a.rows() and c.rows();
			// p must be equal a.cols() and b.rows();
			// n must be equal b.cols() and c.cols();

			int m = a.rows();
			int p = a.cols();
			int n = b.cols();

			c.resize(a.rows(), b.cols());

			for (int i = 0; i < m; ++i)
			{
				for (int j = 0; j < n; ++i)
				{
					typename C::value_type acc = 0;
					for (int k = 0; k < p; ++k)
					{
						acc += a.at(i, k) * b.at(k, j);
					}
				}
			}
		}

		template <class A, class B>
		int square_matrix_inverse(const A& a, B& b)
		{
			// a = input matrix
			// b = result matrix
			// n = number of rows = number of columns in b (n x n)
			int n = a.cols();
			int pivrow;		// keeps track of current pivot row
			int k, i, j;	// k: overall index along diagonal; i: row index; j: col index
			int pivrows[n]; // keeps track of rows swaps to undo at end
			double tmp;		// used for finding max value and making column swaps
			
			b.resize(a.cols(), a.cols());

			for (i = 0 ; i < n; i++)
				for (j = 0 ; j < n; j++)
					b.at(i,j) = a.at(i,j);

			for (k = 0; k < n; k++)
			{
				// find pivot row, the row with biggest entry in current column
				tmp = 0;
				for (i = k; i < n; i++)
				{
					if (abs(b.at(i,k)) >= tmp)	// 'Avoid using other functions inside abs()?'
					{
						tmp = abs(b.at(i,k));
						pivrow = i;
					}
				}

				// check for singular matrix
				if (b.at(pivrow,k) == 0.0f)
				{
					return -1;
				}

				// Execute pivot (row swap) if needed
				if (pivrow != k)
				{
					// swap row k with pivrow
					for (j = 0; j < n; j++)
					{
						tmp = b.at(k,j);
						b.at(k,j) = b.at(pivrow,j);
						b.at(pivrow,j) = tmp;
					}
				}
				pivrows[k] = pivrow;	// record row swap (even if no swap happened)

				tmp = 1.0f / b.at(k,k);	// invert pivot element
				b.at(k,k) = 1.0f;		// This element of input matrix becomes result matrix

				// Perform row reduction (divide every element by pivot)
				for (j = 0; j < n; j++)
				{
					b.at(k,j) = b.at(k,j) * tmp;
				}

				// Now eliminate all other entries in this column
				for (i = 0; i < n; i++)
				{
					if (i != k)
					{
						tmp = b.at(i,k);
						b.at(i,k) = 0.0f; // The other place where in matrix becomes result mat
						for (j = 0; j < n; j++)
						{
							b.at(i,j) = b.at(i,j) - b.at(k,j) * tmp;
						}
					}
				}
			}

			// Done, now need to undo pivot row swaps by doing column swaps in reverse order
			for (k = n - 1; k >= 0; k--)
			{
				if (pivrows[k] != k)
				{
					for (i = 0; i < n; i++)
					{
						tmp = b.at(i,k);
						b.at(i,k) = b.at(i,pivrows[k]);
						b.at(i,pivrows[k]) = tmp;
					}
				}
			}
			return 0;
		};

	}
}

#endif