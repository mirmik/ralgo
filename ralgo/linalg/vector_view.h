#ifndef RALGO_VECTOR_H
#define RALGO_VECTOR_H

#include <stdlib.h>

namespace ralgo 
{
	template<class T>
	class vector_view 
	{
		T* dat;
		int n;

	public:
		using value_type = T;

		vector_view(T* dat, int n) : dat(dat), n(n) {}

		T& operator[](int i) { return *(dat+i); }
		const T& operator[](int i) const { return *(dat+i); }

		T* data() { return dat; }
		const T* data() const { return dat; }
		size_t size() const { return n; }

		T*             begin()       { return dat; } 
		T*             end()         { return dat + n; }
		const T*       begin() const { return dat; } 
		const T* const end()   const { return dat + n; }
	};
}

#endif