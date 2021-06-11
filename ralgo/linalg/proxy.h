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
	};
}

#endif