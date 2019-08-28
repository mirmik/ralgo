#ifndef RALGO_UTIL_BACKPACK_H
#define RALGO_UTIL_BACKPACK_H

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
		Alg current{};

		auto epsilon = 0.0000000001;

		float tgtnorm = target.norm();
		float vnorms[dim];
		Alg vnormalized[dim];

		for (int i = 0; i < dim; ++i) {
			vnorms[i] = vectors[i].norm();
			vnormalized[i] = vectors[i] / vnorms[i]; 
		}

		int32_t iterations = 0;
		while (1)
		{
			iterations++;
			auto delta = target - current;

			if (delta.norm() < epsilon) 
				break;

			for (int i = 0; i < dim; ++i) 
			{
				if (vnorms[i] != 0) 
				{
					auto koeff = vnormalized[i].dot(delta) * 0.5;
					coords[i] += koeff / vnorms[i];
				}
			}

			current = decltype(current)();
			for (int i = 0; i < dim; ++i) 
			{
				current += coords[i] * vectors[i];
			}
		}
	}
}

#endif