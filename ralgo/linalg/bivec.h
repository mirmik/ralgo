#ifndef RALGO_LINALG_BIVEC_H
#define RALGO_LINALG_BIVEC_H

namespace linalg
{

	template<typename T, int N>
	struct bivec
	{
		vec<T, N> a;
		vec<T, N> b;

		bivec(const vec<T, N> _a, const vec<T, N> _b) : a(_a), b(_b) {};

		vec<T, 2 * N> concat()
		{
			vec<T, 2 * N> ret;

			for (int i = 0; i < N; ++i)
			{
				ret[i] = a[i];
				ret[i + N] = b[i];
			}

			return ret;
		}
	};
}

#endif