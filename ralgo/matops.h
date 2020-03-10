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

			for (unsigned int i = 0; i < m; ++i)
				for (unsigned int j = 0; j < n; ++j) 
				{
					a(i, j) = b(i, j);
				}
		}

		template <class M, class VI>
		void copy_from_rows(M& tgt, VI srcit, size_t m) 
		{
			size_t n = srcit->size();

			for (unsigned int i = 0; i < m; ++i) 
			{
				auto& v = *srcit;  
				for (unsigned int j = 0; j < n; ++j) 
				{
					tgt(i,j) = v[j];
				}
				srcit++;
			}
		}

		template <class M, class VI>
		void copy_from_cols(M& tgt, VI srcit, size_t n) 
		{
			size_t m = srcit->size();

			for (unsigned int j = 0; j < n; ++j) 
			{
				auto& v = *srcit;  
				for (unsigned int i = 0; i < m; ++i) 
				{
					tgt(i,j) = v[i];
				}
				srcit++;
			}
		}
	}
}

#endif