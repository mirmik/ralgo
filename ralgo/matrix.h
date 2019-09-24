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

namespace ralgo 
{
	template< class T >
	class matrix_view 
	{
		T* dat;
		int m, n;

	public:
		matrix_view(T* dat, int m, int n) : dat(dat), m(m), n(n) {}

		int size1() const { return m; }
		int size2() const { return n; }

		T& operator()(int i, int j) { return i * n + j; }

		T* begin() { return dat; }
		T* const end() { return dat + m*n; }
		const T* begin() const { return dat; }
		const T* const end() const { return dat + m*n; }
	};
}

#endif