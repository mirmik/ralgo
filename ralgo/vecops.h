#ifndef RALGO_VECTOR_OPERATIONS_H
#define RALGO_VECTOR_OPERATIONS_H

#include <algorithm>
#include <ralgo/util/helpers.h>
#include <igris/dprint.h>

namespace ralgo
{
	template <typename T>
	class vector_view 
	{
		T* dat;
		size_t siz;

	public:
		vector_view(T* dat, size_t siz) : dat(dat), siz(siz) {}

		T* data() { return dat; }
		size_t size() { return siz; }

		T*             begin()       { return dat; } 
		T*             end()         { return dat + siz; }
		const T*       begin() const { return dat; } 
		const T* const end()   const { return dat + siz; }
	};

	namespace vecops
	{
		template<class V>
		auto arange(int start, int stop, int step)
		{
			size_t l = (stop - start) / step;
			int cur = start;
			V r(l);

			for (auto & v : r) { v = cur; cur += step; }

			return r;
		}

		template<class V>
		auto arange(int stop)
		{
			V r(stop);

			for (int i = 0; i < stop; ++i) { r[i] = i; }

			return r;
		}

		// Применить функцию f ко всем элементам массива v. Допускается передача дополнительных аргументов.
		template < template<class> class V, template<class> class R = V, class F, class T, class ... Args >
		auto elementwise(const V<T> & vec, const F& f, Args && ... args)
		-> R<decltype(std::declval<F>()(std::declval<value_t<V<T>>>(), std::declval<Args>() ...))>
		{
			R < decltype(
			    std::declval<F>()(
			        std::declval<value_t<V<T>>>(),
			        std::declval<Args>() ...))
			> ret(std::size(vec));

			auto vit = vec.begin(), eit = vec.end();
			auto cit = ret.begin();

			for (; vit != eit; ++vit, ++cit)
				*cit = f(*vit, std::forward<Args>(args) ...);

			return ret;
		}

		// Создаёт векторизованный вриант функции.
		template<class V, class R = V, typename F, class ... Args>
		constexpr auto vectorize(const F& f, Args && ... args)
		{
			auto lamda = [=](const V & vec){ return elementwise(vec, f, args...); };
			return lamda;
		}

		template <template<class C> class V, class T> auto abs(const V<T>& obj) { return elementwise(obj, std::abs<scalar_t<T>>); }
		template <template<class C> class V, class T> auto real(const V<T>& obj) { return elementwise(obj, [](const auto & c) {return c.real();}); }
		template <template<class C> class V, class T> auto imag(const V<T>& obj) { return elementwise(obj, [](const auto & c) {return c.imag();}); }

		template <typename T>
		T reverse(const T& src)
		{
			T dst(src.size());
			auto sit = src.begin();
			auto eit = src.end();
			auto dit = dst.rbegin();

			for (; sit != eit;) { *dit++ = *sit++; }

			return dst;
		}

		template <typename T>
		T slice(const T& src, size_t start, size_t size, size_t stride)
		{
			T dst(size);

			size_t idx = start;

			for (int i = 0; i < size; ++i)
			{
				dst[i] = src[idx];
				idx += stride;
			}

			return dst;
		}

		template <typename V>
		auto norm(const V& vec) -> typename V::value_type
		{
			double res = 0;

			for (auto& val : vec) { auto a = std::abs(val); res += a * a; }

			return sqrt(res);
		}

		template <class V, class S> void add(const V& v, S s) { V r(std::size(v)); for (int i = 0; i < std::size(v); ++i) r[i] = v[i] + s; return r; }
		template <class V, class S> void sub(const V& v, S s) { V r(std::size(v)); for (int i = 0; i < std::size(v); ++i) r[i] = v[i] - s; return r; }
		template <class V, class S> void mul(const V& v, S s) { V r(std::size(v)); for (int i = 0; i < std::size(v); ++i) r[i] = v[i] * s; return r; }
		template <class V, class S> void div(const V& v, S s) { V r(std::size(v)); for (int i = 0; i < std::size(v); ++i) r[i] = v[i] / s; return r; }



		namespace inplace
		{
			// Применить функцию f ко всем элементам массива v. Допускается передача дополнительных аргументов.
			template <class V, class F, class ... A>
			V& elementwise(V& v, F f, A&& ... a)
			{
				for (auto & s : v) s = f(s, std::forward<A>(a) ...);
				return v;
			}

			template <class V, class S> V& add(V& vec, S m) { for (auto& val : vec) val += m; return vec; }
			template <class V, class S> V& sub(V& vec, S m) { for (auto& val : vec) val -= m; return vec; }
			template <class V, class S> V& mul(V& vec, S m) { for (auto& val : vec) val *= m; return vec; }
			template <class V, class S> V& div(V& vec, S m) { for (auto& val : vec) val /= m; return vec; }

			template <class V> V& conj(V& vec) { for (auto& val : vec) val = std::conj(val); return vec; }

			template <class V> V& sin(V& vec) { return elementwise(vec, ralgo::sin<value_t<V>>); }
		}

		template <class VI, class WI, class RI>
		void stamp_union(VI vit, const VI vend, WI wit, const WI wend, RI rit) 
		{
			while(vit != vend && wit != wend) 
			{
				bool cmp = *vit < *wit;

				if (cmp) { *rit++ = *vit++; } 
				else     { *rit++ = *wit++; }
			}

			while (vit != vend) { *rit++ = *vit++; } 
			while (wit != wend) { *rit++ = *wit++; } 
		}

		template <class V, class W, class R>
		void stamp_union_window(const V& v, const W& w, R writer) 
		{
			auto vit = std::lower_bound(v.begin(), v.end(), w[0]);
			auto wit = w.begin();
			auto vend = std::upper_bound(v.begin(), v.end(), w[w.size()-1]);
			auto wend = w.end();

			return stamp_union(vit, vend, wit, wend, writer);			
		}
	}

	using vecops::elementwise;
	using vecops::vectorize;

	namespace inplace 
	{
		using vecops::inplace::elementwise;
		//using vecops::inplace::vectorize;
	}
}

#endif