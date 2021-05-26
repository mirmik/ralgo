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

#include <ralgo/linalg/vector_view.h>
#include <ralgo/log.h>

namespace ralgo
{
	template <class T>
	struct row_order
	{
		static T& at(T* data, int i, int j, int stride) { return *(data + i * stride + j); }
		static int stride(int rows, int cols) { return cols; }
		static vector_view<T> sect(T* data, int i, int rows, int cols) { return { data + i * cols, cols }; }
	};

	template <class T>
	struct collumn_order
	{
		static T& at(T* data, int i, int j, int stride) { return *(data + j * stride + j); }
		static int stride(int rows, int cols) { return rows; }
		static vector_view<T> sect(T* data, int i, int rows, int cols) { return { data + i * rows, rows }; }
	};

	template< class T, class O = row_order<T>>
	class matrix_view
	{
		T*  _data;
		int _rows;
		int _cols;
		int _stride;

		O accessor;

	public:
		using value_type = T;

		matrix_view(T* data, int rows, int cols) :
			_data(data), _rows(rows), _cols(cols), _stride(accessor.stride(rows, cols))
		{}

		matrix_view(const matrix_view& oth) :
			_data(oth._data), _rows(oth._rows), _cols(oth._cols), _stride(oth._stride)
		{};

		size_t size1() const { return _rows; }
		size_t size2() const { return _cols; }

		T& at(int i, int j)
		{
			return accessor.at(_data, i, j, _stride);
		}

		const T& at(int i, int j) const
		{
			return accessor.at(_data, i, j, _stride);
		}

		T& operator()(int i, int j)
		{
			return at(i, j);
		}

		const T& operator()(int i, int j) const
		{
			return at(i, j);
		}

		ralgo::vector_view<T> operator[](int i) { return accessor.sect(_data, i, _rows, _cols); }
		const ralgo::vector_view<T> operator[](int i) const { return accessor.sect(_data, i, _rows, _cols); }

		// vecops compatible. Убрать, если потребуются смещения.
		T* begin() { return _data; }
		T* const end() { return _data + _cols * _rows; } // Stride ?
		const T* begin() const { return _data; }
		const T* const end() const { return _data + _rows * _cols; } // Stride ?

		template<class M>
		matrix_view& operator = (const M& oth)
		{
			for (int i = 0; i < _rows; ++i)
				for (int j = 0; j < _cols; ++j)
					operator()(i, j) = oth(i, j);

			return *this;
		}

		matrix_view& operator = (const matrix_view& oth)
		{
			for (int i = 0; i < _rows; ++i)
				for (int j = 0; j < _cols; ++j)
					operator()(i, j) = oth(i, j);

			return *this;
		}
	};

	template <class T> using matrix_view_co = matrix_view<T, collumn_order<T>>;
	template <class T> using matrix_view_ro = matrix_view<T, row_order<T>>;
}

#endif