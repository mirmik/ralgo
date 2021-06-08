#ifndef RALGO_INTERPOLATE_H
#define RALGO_INTERPOLATE_H

#include <iterator>
#include <nos/print.h>

namespace ralgo
{
	template <class A, class B, class K>
	auto lerp(A a, B b, K k)
	{
		return (1 - k) * a + k * b;
	}

	template <class A, class B, class U>
	auto lerpkoeff(A left, B right, U target)
	{
		return ( target - left ) / ( right - left );
	}

	template <class T>
	class linspace
	{
		class linspace_iterator : public std::iterator<std::bidirectional_iterator_tag, T, T, T*, T&>
		{
			linspace * ls;
			int p;

		public:
			linspace_iterator(linspace* ls, int p) : ls(ls), p(p) {}
			T operator * () { return ls->operator[](p); }
			linspace_iterator & operator++() { ++p; return *this; }

			bool operator==(const linspace_iterator & oth) { return p == oth.p && ls == oth.ls; }
			bool operator!=(const linspace_iterator & oth) { return p != oth.p || ls != oth.ls; }
		};

		T a, b;
		int points;

	public:
		linspace(T _a, T _b, int _points, bool endpoint = true) :
			a(_a), b(_b), points(_points)
		{
			if (!endpoint)
			{
				T k = (T)(points-1) / (T)(points);
				b = lerp(a,b,k);
			}
		}

		T step()
		{
			return (b - a) / (points - 1);
		}

		T operator[](int p)
		{
			T koeff = (T)p / (T)(points - 1);
			return lerp(a, b, koeff);
		}

		int size() const { return points; }

		linspace_iterator begin() { return linspace_iterator(this, 0); }
		linspace_iterator end() { return linspace_iterator(this, points); }
	};
}

#endif