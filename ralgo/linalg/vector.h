#ifndef RALGO_LINALG_VECTOR_H
#define RALGO_LINALG_VECTOR_H

#include <vector>
#include <iostream>

namespace ralgo 
{
	template <class T, class Alloc = std::allocator<T>>
	using vector = std::vector<T, Alloc>;
}

namespace std 
{
	template<class T>
	std::ostream & operator << (std::ostream & os, const ralgo::vector<T> & vec) 
	{
		os << '{';
		for (int i = 0; i < vec.size()-1; ++i) 
		{
			os << vec[i] << ',';
		}
		if (vec.size() != 0)
			os << vec[vec.size()-1];
		os << '}';
		return os;		
	}
}

#endif

