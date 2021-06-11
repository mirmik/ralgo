#ifndef RALGO_LINALG_TRANSPOSED_MATRIX_VIEW_H
#define RALGO_LINALG_TRANSPOSED_MATRIX_VIEW_H

namespace ralgo
{
	template <class M>
	class transposed_matrix_proxy
	{
		M& mat;

	public:
		transposed_matrix_proxy(M& mat) : mat(mat) {}

		auto & at(int i, int j) { return mat.at(j, i); }

		const auto & at(int i, int j) const { return mat.at(j, i); }

		int rows() const { return mat.cols(); }
		int cols() const { return mat.rows(); }

		//ralgo::vector_view<typename M::value_type>
		auto row(int i) { return mat.col(i); }
		auto col(int i) { return mat.row(i); }
	};

	template <class V>
	class inverted_diagonal_proxy
	{
		V& vec;

	public:
		inverted_diagonal_proxy(V& vec) : vec(vec) {}

		auto at(int i, int j) const
		{
			return i == j ? 1. / vec[i] : 0;
		}

		int rows() const { return vec.size(); }
		int cols() const { return vec.size(); }
	};

}

#endif