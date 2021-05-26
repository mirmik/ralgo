#ifndef RALGO_MATOPS_H
#define RALGO_MATOPS_H

#include "assert.h"

#include <utility>
#include <iterator>

#include <ralgo/linalg/vecops_base.h>
#include <ralgo/linalg/vecops.h>

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


		template <class A, class B, class C>
		void multiply(const A& a, const B& b, C& c)
		{
			// m must be equal a.rows() and c.rows();
			// p must be equal a.cols() and b.rows();
			// n must be equal b.cols() and c.cols();

			int m = a.rows();
			int p = a.cols();
			int n = b.cols();

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
	}
}

#endif