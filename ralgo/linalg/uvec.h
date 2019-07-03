#ifndef RALGO_LINALG_UVEC_H
#define RALGO_LINALG_UVEC_H

namespace linalg
{
	template<typename T, int M>
	struct uvec : public vec<T, M>
	{
		uvec(const vec<T, M>& v) : vec<T, M>(normalize(v)) {}
		uvec(const uvec<T, M>& u) : vec<T, M>(u) {}
	};
}

#endif