#ifndef RALGO_VECTOR_OPERATIONS_H
#define RALGO_VECTOR_OPERATIONS_H

#include <ralgo/util/helpers.h>
#include <igris/dprint.h>

namespace ralgo
{
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
		template <template<class C> class V, class T, class F, class ... A>
		auto elementwise(const V<T>& v, F f, A&& ... a) -> V<decltype(std::declval<F>()(std::declval<typename V<T>::value_type>()))>
		{
			V<scalar_t<T>> r(std::size(v));
			scalar_t<T> * p = r.data();

			for (const auto & s : v)
				*p++ = f(s, std::forward<A>(a) ...);

			return r;
		}

		template <template<class C> class V, class T> auto abs(const V<T>& obj) -> V<scalar_t<T>> { return elementwise(obj, std::abs<scalar_t<T>>); }
		template <template<class C> class V, class T> auto real(const V<T>& obj) -> V<scalar_t<T>> { return elementwise(obj, [](const auto & c) {return c.real();}); }
		template <template<class C> class V, class T> auto imag(const V<T>& obj) -> V<scalar_t<T>> { return elementwise(obj, [](const auto & c) {return c.imag();}); }

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

		template <template<class> class V, template<class> class R = V, class F, class T, class ... Args>
		auto vectorize(const V<T> & vec, F& f, Args&& ... args) -> R<T>
		{
			R<T> ret(std::size(vec)); auto vit = vec.begin(), eit = vec.end(); auto cit = ret.begin();
			for (;vit != eit; ++vit, ++cit) *cit = f(*vit, std::forward<Args>(args) ...);
			return ret;
		}

		namespace inplace
		{
			// Применить функцию f ко всем элементам массива v. Допускается передача дополнительных аргументов.
			template <class V, class F, class ... A>
			void elementwise(V& v, F f, A&& ... a)
			{
				for (auto & s : v) s = f(s, std::forward<A>(a) ...);
			}

			template <class V, class S> void add(V& vec, S m) { for (auto& val : vec) val += m; }
			template <class V, class S> void sub(V& vec, S m) { for (auto& val : vec) val -= m; }
			template <class V, class S> void mul(V& vec, S m) { for (auto& val : vec) val *= m; }
			template <class V, class S> void div(V& vec, S m) { for (auto& val : vec) val /= m; }

			template <class V> void conj(V& vec) { for (auto& val : vec) val = std::conj(val); }

			template <class V> void sin(V& vec) { elementwise(vec, ralgo::sin<value_t<V>>); }
		}
	}

	using vecops::vectorize;
}

#endif