#ifndef RALGO_VECTOR_VIEW_H
#define RALGO_VECTOR_VIEW_H

/**
	@file
*/

namespace ralgo 
{
	template <class T>
	class vector_view_iterator 
	{
		T* dat;
		int stride;

	public:
		vector_view_iterator(T* dat) : dat(dat), stride(1) {}
		vector_view_iterator(T* dat, int stride) : dat(dat), stride(stride) {}

		vector_view_iterator & operator++() 
		{
			dat += stride;
			return *this;
		}

		vector_view_iterator operator++(int) 
		{
			vector_view_iterator ret = *this;
			dat += stride;
			return ret;
		}

		T& operator *() { return *dat; }
		T& operator ->() { return *dat; }

		bool operator != (const vector_view_iterator & oth) const
		{
			return dat != oth.dat || stride != stride; 
		}

		bool operator == (const vector_view_iterator & oth) const
		{
			return dat == oth.dat && stride == stride; 
		}

	};

	template<class T>
	class vector_view 
	{
		T* dat;
		int n;
		int stride;

	public:
		using iterator = vector_view_iterator<T>;
		using value_type = T;

		void resize(int sz) { n = sz; }

		vector_view() : dat(nullptr), n(0), stride(0) {}
		vector_view(T* dat, int n) : dat(dat), n(n), stride(1) {}
		vector_view(T* dat, int n, int stride) : dat(dat), n(n), stride(stride) {}

		T& operator[](int i) { return *(dat+i*stride); }
		const T& operator[](int i) const { return *(dat+i*stride); }

		T* data() { return dat; }
		const T* data() const { return dat; }
		size_t size() const { return n; }

		iterator             begin()       { return {dat, stride}; } 
		iterator             end()         { return {dat + n * stride, stride}; }
		const iterator       begin() const { return {dat, stride}; } 
		const iterator       end()   const { return {dat + n * stride, stride}; }
	};
}

#endif