#ifndef RALGO_LINALG_OPTIMIZE_H
#define RALGO_LINALG_OPTIMIZE_H

#include <malgo/matrix.h>
#include <malgo/nrecipes/svd.h>
#include <malgo/vector.h>

namespace ralgo 
{
	namespace vecops
	{
		template <typename T, int M>
		malgo::vector<T> backpack(linalg::vec<T, M> need, std::vector<vec<T, M>> sens)
		{
			malgo::matrix<T> m(sens);
			m = transpose(m);
			malgo::vecview<T> b(&need[0], 6);
			malgo::vector<T> x(sens.size());
			malgo::SVD<malgo::matrix<T>> svd(m);

			try
			{
				svd.solve(b, x);
			}
			catch (const char *c)
			{
				return {};
			}

			return x;
		}
	}
}

#endif