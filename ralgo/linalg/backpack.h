#ifndef RALGO_UTIL_BACKPACK_H
#define RALGO_UTIL_BACKPACK_H

#include <nos/trace.h>
#include <igris/util/iteration_counter.h>

#include <ralgo/linalg/solve.h>

namespace ralgo
{
	template <class T, class Alg>
	void gradient_backpack
	(
	    T* coords,
	    const Alg& target,
	    Alg* vectors,
	    int dim
	)
	{
		//TRACE();
		Alg current;

		// Точность может существенно влиять на время.
		auto epsilon = 0.0000001;

		float tgtnorm = target.norm();
		float vnorms[dim];
		Alg vnormalized[dim];

		for (int i = 0; i < dim; ++i) {
			vnorms[i] = vectors[i].norm();
			vnormalized[i] = vectors[i] / vnorms[i]; 
		}

		int32_t iterations = 0;
		Alg lastcurrent;
		float lastnorm;
		while (1)
		{
			iterations++;
			auto delta = target - current;

			auto norm = delta.norm(); 
			//if (lastnorm > norm) { 
			//if (predelta == delta)
			if (norm < epsilon) {
			//	current = lastcurrent;
				break;
			}

			//predelta = delta;
			lastnorm = norm;

			for (int i = 0; i < dim; ++i) 
			{
				if (vnorms[i] != 0) 
				{
					auto koeff = vnormalized[i].dot(delta) * 0.9;
					coords[i] += koeff / vnorms[i];
				}
			}

			//PRINT(delta);
			lastcurrent = current;
			current = decltype(current)();
			for (int i = 0; i < dim; ++i) 
			{
				current += coords[i] * vectors[i];
			}
		}
	}

		template <class T, class Alg>
	void very_naive_backpack
	(
	    T* coords,
	    const Alg& target,
	    Alg* vectors,
	    int dim
	)
	{
		float vnorms[dim];
		Alg vnormalized[dim];

		for (int i = 0; i < dim; ++i) {
			vnorms[i] = vectors[i].norm();
			vnormalized[i] = vectors[i] / vnorms[i]; 
		}

		for (int i = 0; i < dim; ++i) 
		{
			coords[i] = target.dot(vnormalized[i]);
		}
	}

	template <class T, class Alg>
	void svd_backpack(    
		T* coords,
	    const Alg& target,
	    Alg* vectors,
	    int count_of_vectors
	) 
	{
		//double result[count_of_vectors];
		ralgo::vector_view<T> result(coords, count_of_vectors);

		// Размер вектора целей.
		auto tsize = target.size();

		T matdata[tsize * count_of_vectors];
		ralgo::matrix_view<T> mat(matdata, tsize, count_of_vectors);

		ralgo::matops::copy_from_cols(mat, igris::array_view<Alg>(vectors, count_of_vectors));

		// Нагрузка на стэк.
		ralgo::solve_linear_equation_system(result, mat, target);
	};

	template <class C, class T, class M>
	void svd_backpack(    
		C& coords,
	    const T& target,
		const M& matrix
	) 
	{
		ralgo::solve_linear_equation_system(coords, matrix, target);
	};
}

#endif