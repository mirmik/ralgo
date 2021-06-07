#ifndef RALGO_LINALG_VECTOR_H
#define RALGO_LINALG_VECTOR_H

#include <vector>

namespace ralgo 
{
	template <class T, class Alloc>
	using vector = std::vector<T, Alloc>;
}

#endif

