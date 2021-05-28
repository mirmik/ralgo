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
		static int stride(int rows, int cols) { (void) rows; return cols; }
		static vector_view<T> sect(T* data, int i, int rows, int cols) { return { data + i * cols, cols }; }
	};

	template <class T>
	struct collumn_order
	{
		static T& at(T* data, int i, int j, int stride) { return *(data + j * stride + i); }
		static int stride(int rows, int cols) { (void) cols; return rows; }
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

		matrix_view() : _data(nullptr), _rows(0), _cols(0), _stride(0) {}

		matrix_view(T* data, int rows, int cols, int stride) :
			_data(data), _rows(rows), _cols(cols), _stride(stride)
		{}

		matrix_view(T* data, int rows, int cols) :
			_data(data), _rows(rows), _cols(cols), _stride(accessor.stride(rows, cols))
		{}

		matrix_view(const matrix_view& oth) :
			_data(oth._data), _rows(oth._rows), _cols(oth._cols), _stride(oth._stride)
		{};

		matrix_view(const std::initializer_list<T> & lst, int rows, int cols)
			: matrix_view((T*) std::begin(lst), rows, cols)
		{}

		int rows() const { return _rows; }
		int cols() const { return _cols; }

		size_t size1() const { return _rows; }
		size_t size2() const { return _cols; }

		T* data() { return _data; }
		const T* data() const { return _data; }

		void resize(int rows, int cols)
		{
			_rows = rows;
			_cols = cols;
			_stride = accessor.stride(rows, cols);
		}

		T& at(int i, int j) { return accessor.at(_data, i, j, _stride); }
		const T& at(int i, int j) const { return accessor.at(_data, i, j, _stride); }

		T& operator()(int i, int j) { return at(i, j); }
		const T& operator()(int i, int j) const { return at(i, j); }

		ralgo::vector_view<T> operator[](int i) { return accessor.sect(_data, i, _rows, _cols); }
		const ralgo::vector_view<T> operator[](int i) const { return accessor.sect(_data, i, _rows, _cols); }

		// vecops compatible. Убрать, если потребуются смещения.
		T* begin() { return _data; }
		T* end() { return _data + _cols * _rows; } // Stride ?
		const T* begin() const { return _data; }
		const T* end() const { return _data + _rows * _cols; } // Stride ?

		matrix_view& operator = (const matrix_view& oth)
		{
			_data = (T*)oth.data();
			_rows = oth._rows;
			_cols = oth._cols;
			_stride = accessor.stride(_rows, _cols);
			return *this;
		}
	};

	template <class T> using matrix_view_co = matrix_view<T, collumn_order<T>>;
	template <class T> using matrix_view_ro = matrix_view<T, row_order<T>>;

	template<class T, class O>
	std::ostream& operator<<(std::ostream& os, const matrix_view<T, O>& m)
	{
		if (m.rows() == 0 || m.cols() == 0)
		{
			os << "(null_matrix)";
			return os;
		}

		for (int i = 0; i < m.rows(); ++i)
		{
			for (int j = 0; j < m.cols(); ++j)
			{
				os << m.at(i, j) << " ";
			}
			os << std::endl;
		}

		return os;
	}
}

#endif