#ifndef RALGO_SPACE_SCREW_H
#define RALGO_SPACE_SCREW_H

#include <ralgo/linalg/linalg.h>

namespace ralgo
{
	template< class T, int N >
	struct screw;

	template <class T>
	using screw2 = screw<T, 2>;

	template <class T>
	using screw3 = screw<T, 3>;

	template <class T>
	struct screw<T, 3>
	{
		linalg::vec<T, 3> ang;
		linalg::vec<T, 3> lin;

		screw(const linalg::vec<T, 3>& ang, const linalg::vec<T, 3>& lin);

		screw operator * (float koeff)
		{
			return { ang * koeff, lin * koeff };
		}

		T rotation() { return ang; }
		linalg::vec<T, 3> translation() { return lin; }
	};

	template <class T>
	struct screw<T, 2>
	{
		union
		{
			T arr[3];
			struct
			{
				T ang;
				linalg::vec<T, 2> lin;
			};
		};

		screw(T ang, const linalg::vec<T, 2>& lin) : ang(ang), lin(lin) {}
		screw() : ang(0), lin(0, 0) {}

		constexpr size_t size() const { return 3; }
		T& operator [](int i) { return arr[i]; }
		const T& operator [](int i) const { return arr[i]; }

		template<class U>
		screw(const screw2<U>& oth) : ang(oth.ang), lin(oth.lin.x, oth.lin.y) {}

		template<class U>
		screw& operator = (const screw2<U>& oth)
		{
			ang = oth.ang;
			lin.x = oth.lin.x;
			lin.y = oth.lin.y;
			return *this;
		}

		screw operator * (float koeff)
		{
			return { ang * koeff, lin * koeff };
		}

		T rotation() { return ang; }
		linalg::vec<T, 2> translation() { return lin; }

		T norm() const { return sqrt(ang * ang + lin.x * lin.x + lin.y * lin.y); }

		T dot(const screw& o) const { return ang * o.ang + lin.x * o.lin.x + lin.y * o.lin.y; }

		bool operator == (const screw& b)
		{ return ang == b.ang && lin == b.lin; }

		ssize_t print_to(nos::ostream& os) const
		{ return nos::fprint_to(os, "({},{})", ang, lin); }
	};

	template<class T, int N>
	screw<T, N> operator - (const screw<T, N>& a, const screw<T, N>& b)
	{ return { a.ang - b.ang, a.lin - b.lin }; }

	template<class T, int N>
	screw<T, N> operator + (const screw<T, N>& a, const screw<T, N>& b)
	{ return { a.ang + b.ang, a.lin + b.lin }; }

	template<class T, int N>
	screw<T, N>& operator += (screw<T, N>& a, const screw<T, N>& b)
	{
		a.ang += b.ang;
		a.lin += b.lin;
		return a;
	}

	template<class T, int N, class O>
	screw<T, N> operator / (const screw<T, N>& a, const O& b)
	{ return { a.ang / b, a.lin / b }; }

	template<class T, int N, class O>
	screw<T, N> operator * (const O& b, const screw<T, N>& a)
	{ return { b * a.ang, b * a.lin }; }

	template<class C, class T, int N>
	std::basic_ostream<C> & operator << (
	    std::basic_ostream<C> & out,
	    const ralgo::screw<T, N> & tr)
	{
		return out << '{' << tr.ang << ',' << tr.lin << '}';
	}
}

#endif