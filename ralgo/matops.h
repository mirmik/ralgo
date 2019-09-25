#ifndef RALGO_MATOPS_H
#define RALGO_MATOPS_H

namespace ralgo
{
	namespace matops
	{
		template<class A, class B>
		void copy(A& a, const B& b)
		{
			size_t m = a.size1();
			size_t n = a.size2();

			for (int i = 0; i < m; ++i)
				for (int j = 0; j < n; ++j) 
				{
					a(i, j) = b(i, j);
				}
		}
	}
}

#endif