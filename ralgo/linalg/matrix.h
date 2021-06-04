#ifndef RALGO_LINALG_MATRIX_H
#define RALGO_LINALG_MATRIX_H

#include <memory>
#include <ralgo/linalg/matrix_view.h>
#include <nos/print.h>

namespace ralgo 
{
	template <class T, class O = ralgo::row_order<T>, class Alloc = std::allocator<T>>
	class matrix : public matrix_view<T, O>
	{
		using parent = matrix_view<T, O>;
		Alloc alloc;

	public:
		matrix() : parent() {
			nos::println("matrix()");
		}

		matrix(const matrix & oth) 
			: parent()
		{
			nos::println("matrix(const&)");
			int rows = oth.rows();
			int cols = oth.cols();

			resize(rows, cols);
			for (int i = 0; i < rows; ++i)
				for (int j = 0; j < cols; ++j)
					this->at(i,j) = oth.at(i,j);
		}

		matrix(int r, int c) : 
			parent(nullptr, r, c)
		{
			nos::println("matrix(r,c)");
			nos::println("allocate");
			this->_data = alloc.allocate(r * c);
		}

		void resize(int r, int c) 
		{
			nos::println("resize");
			invalidate();

			this->_data = alloc.allocate(r * c);
			parent::resize(r, c);
		}

		~matrix() 
		{
			nos::println("~matrix()");
			invalidate();
		}

		void invalidate() 
		{
			nos::println("invalidate");
			if (this->_data) {
				nos::println("deallocate");
				alloc.deallocate(this->_data, this->_rows * this->_cols);	
			}
		
			parent::release();
		}

		matrix & operator=(const matrix& oth) 
		{
			nos::println("operator = matrix");
			int rows = oth.rows();
			int cols = oth.cols();

			resize(rows, cols);
			for (int i = 0; i < rows; ++i)
				for (int j = 0; j < cols; ++j)
					this->at(i,j) = oth.at(i,j);

			return *this;
		}

		template <class M>
		matrix & operator=(const M& oth) 
		{
			nos::println("operator = M");
			int rows = oth.rows();
			int cols = oth.cols();

			resize(rows, cols);
			for (int i = 0; i < rows; ++i)
				for (int j = 0; j < cols; ++j)
					this->at(i,j) = oth.at(i,j);

			return *this;
		}
	};
}

#endif