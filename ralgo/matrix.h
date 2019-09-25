#ifndef RALGO_MATRIX_H
#define RALGO_MATRIX_H

/**
	Соглашение об интерфейсе матриц в библиотеке ralgo
	взято от матриц boost::matrix.

	Основные операции -
	Геттер - matrix(i,j)
	Количество строк - matrix.size1() 
	Количество столбцов - matrix.size2()
*/

#include <ralgo/vector.h>
#include <ralgo/fault.h>
#include <nos/print.h>

namespace ralgo 
{
	template< class T >
	class matrix_view 
	{
		T* dat;
		size_t m, n;

	public:
		using value_type = T;

		matrix_view(T* dat, int m, int n) : dat(dat), m(m), n(n) {}
		matrix_view(const matrix_view& oth) : dat(oth.dat), m(oth.m), n(oth.n) {};

		size_t size1() const { return m; }
		size_t size2() const { return n; }

		T& operator()(int i, int j) { return *(dat + (i * n + j)); }
		const T& operator()(int i, int j) const { return *(dat + (i * n + j)); }

		ralgo::vector_view<T> operator[](int i) { return { dat+i*n, n }; }
		const ralgo::vector_view<T> operator[](int i) const { return { dat+i*n, n }; }

		// vecops compatible. Убрать, если потребуются смещения.
		T* begin() { return dat; }
		T* const end() { return dat + m*n; }
		const T* begin() const { return dat; }
		const T* const end() const { return dat + m*n; }

		template<class M>
		matrix_view& operator = (const M& oth) 
		{
			nos::println("operator = ");

			if (oth.size1() != size1() || oth.size2() != size2())
				ralgo::fault("incompatible matrices");

			for (int i = 0; i < m; ++i)
				for (int j = 0; j < n; ++j)
					operator()(i,j) = oth(i,j);

			return *this;
		}

		matrix_view& operator = (const matrix_view& oth) 
		{
			nos::println("operator = ");

			if (oth.size1() != size1() || oth.size2() != size2())
				ralgo::fault("incompatible matrices");

			for (int i = 0; i < m; ++i)
				for (int j = 0; j < n; ++j)
					operator()(i,j) = oth(i,j);

			return *this;
		}
	};
}

#endif