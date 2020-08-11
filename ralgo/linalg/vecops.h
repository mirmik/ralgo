#ifndef RALGO_VECTOR_OPERATIONS_H
#define RALGO_VECTOR_OPERATIONS_H

#include <assert.h>
#include <algorithm>
#include <iterator>
#include <ralgo/util/helpers.h>

#include <igris/dprint.h>
#include <igris/util/signature.h>

#include <ralgo/linalg/vecops_base.h>

namespace ralgo
{
	IGRIS_SIGNATURE_ATTRIBUTE_CHECKER(has_reserve, reserve);

	namespace vecops
	{
		template < class R=void, class V >
		defvec_t<R,V> list(const V& u) 
		{
			defvec_t<R,V> ret;

#if __cplusplus >= 201703L
			if constexpr (has_reserve<defvec_t<R,V>>()) 
				ret.reserve(u.size());
#endif

			for (const auto& a : u) 
				ret.push_back(a);

			return ret;
		}

		template<class V>
		void fill(V& arr, const value_t<V>& val)
		{
			for (auto & a : arr) 
				a = val;
		}

		// построить целочисленный вектор арифметической прогрессии [start;stop) с шагом step.
		// TODO: дополнить нецелочисленным вариантом
		template<class V>
		auto arange(int start, int stop, int step)
		{
			size_t l = (stop - start) / step;
			int cur = start;
			V r(l);

			for (auto & v : r) { v = cur; cur += step; }

			return r;
		}

		template<class V=void>
		defvec_of_t<V,int> arange(int stop)
		{
			defvec_of_t<V,int> r(stop);

			for (int i = 0; i < stop; ++i) { r[i] = i; }

			return r;
		}

		// Построить линейное пространство аля numpy.
		template<class R>
		auto linspace(double s, double f, int n)
		{
			R r(n);
			double k;

			for (int i = 0; i < n; ++i)
			{
				k = (double)i / ((double)n - 1);
				r[i] = s * (1 - k) + f * k;
			}

			return r;
		}



		// Создаёт векторизованный вриант функции.
		template<class V, class R = V, typename F, class ... Args>
		constexpr auto vectorize(const F& f, Args && ... args)
		{
			auto lamda = [ = ](const V & vec) { return elementwise<R>(f, vec, args...); };
			return lamda;
		}

		// Векторизованные операции над комплексными числами.
		template <class R = void, class A> defsame_t<R, A> abs(const A& obj) { return elementwise<R>(ralgo::op_abs(), obj); }
		template <template<class C> class V, class T> auto real(const V<T>& obj) { return elementwise<V<T>>([](const auto & c) {return c.real();}, obj); }
		template <template<class C> class V, class T> auto imag(const V<T>& obj) { return elementwise<V<T>>([](const auto & c) {return c.imag();}, obj); }

		// Вернуть реверс вектора src
		template <typename T> T reverse(const T& src)
		{
			T dst(src.size());
			auto sit = src.begin();
			auto eit = src.end();
			auto dit = dst.rbegin();

			for (; sit != eit;) { *dit++ = *sit++; }

			return dst;
		}

		template <typename T>
		T slice(const T& src, size_t start, size_t size, size_t stride = 1)
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

		// Вычислить длину вектора по евклидовой метрике.
		template <typename V>
		auto norm(const V& vec) -> typename V::value_type
		{
			double res = 0;

			for (const auto& val : vec) { auto a = std::abs(val); res += a * a; }

			return sqrt(res);
		}
		template <typename V>
		auto length(const V& vec) -> typename V::value_type
		{
			return norm(vec);
		}

		// Вычислить расстояние между кортежами равного размера по
		// евклидовой метрике.
		template <class R = double, class A, class B>
		R distance2(const A& a, const B& b)
		{
			R accum = 0;

			for (unsigned int i = 0; i < a.size(); ++i)
			{
				auto diff = a[i] - b[i];
				accum += diff * diff;
			}

			return accum;
		}

		// Вычислить расстояние между кортежами равного размера по
		// евклидовой метрике.
		template <class R = double, class A, class B>
		R distance(const A& a, const B& b)
		{
			return sqrt(distance2<R, A, B>(a, b));
		}

		template <class R, class A>
		R cast(const A& a)
		{
			return elementwise<R>([](auto x) { return x; }, a);
		}

		template <class A> bool all(const A& a) { return fold(op_and(), true, a); }
		template <class A> bool any(const A& a) { return fold(op_or(), false, a); }

		//template <class A, class B> bool equal_all(const A& a, const B& b)
		//{ return fold(ralgo::op_not_eq(), a, veciter(b)); }

		template <class A, class B> bool equal_all(const A& a, const B& b) { return boolean_all(ralgo::op_eq(), a, b); }
		template <class A, class B> bool equal_any(const A& a, const B& b) { return boolean_any(ralgo::op_eq(), a, b); }

		template <class R = void, class A, class B> defsame_t<R, A> add_vs(const A& a, B b) { return elementwise<R>(ralgo::op_add(), a, b); }
		template <class R = void, class A, class B> defsame_t<R, A> sub_vs(const A& a, B b) { return elementwise<R>(ralgo::op_sub(), a, b); }
		template <class R = void, class A, class B> defsame_t<R, A> mul_vs(const A& a, B b) { return elementwise<R>(ralgo::op_mul(), a, b); }
		template <class R = void, class A, class B> defsame_t<R, A> div_vs(const A& a, B b) { return elementwise<R>(ralgo::op_div(), a, b); }

		template <class R = void, class A, class B> defsame_t<R, A> add_vv(const A& a, const B& b) { return elementwise2<R>(ralgo::op_add(), a, b); }
		template <class R = void, class A, class B> defsame_t<R, A> sub_vv(const A& a, const B& b) { return elementwise2<R>(ralgo::op_sub(), a, b); }
		template <class R = void, class A, class B> defsame_t<R, A> mul_vv(const A& a, const B& b) { return elementwise2<R>(ralgo::op_mul(), a, b); }
		template <class R = void, class A, class B> defsame_t<R, A> div_vv(const A& a, const B& b) { return elementwise2<R>(ralgo::op_div(), a, b); }

		template <class A, class B, class C> void add_vv_to(C& c, const A& a, const B& b) { return elementwise2_to(c, ralgo::op_add(), a, b); }
		template <class A, class B, class C> void sub_vv_to(C& c, const A& a, const B& b) { return elementwise2_to(c, ralgo::op_sub(), a, b); }
		template <class A, class B, class C> void mul_vv_to(C& c, const A& a, const B& b) { return elementwise2_to(c, ralgo::op_mul(), a, b); }
		template <class A, class B, class C> void div_vv_to(C& c, const A& a, const B& b) { return elementwise2_to(c, ralgo::op_div(), a, b); }

		namespace inplace
		{
			// Применить функцию f ко всем элементам массива v. Допускается передача дополнительных аргументов.
			template <class V, class F, class ... A>
			V& elementwise(V& v, F f, A&& ... a)
			{
				for (auto & s : v) s = f(s, std::forward<A>(a) ...);
				return v;
			}

			template <class V> void clean(V& v)
			{
				elementwise(v, [](auto & a) {return typename V::value_type();});
			}

			template <class V, class S> V& add(V& vec, S m) { for (auto& val : vec) val += m; return vec; }
			template <class V, class S> V& sub(V& vec, S m) { for (auto& val : vec) val -= m; return vec; }
			template <class V, class S> V& mul(V& vec, S m) { for (auto& val : vec) val *= m; return vec; }
			template <class V, class S> V& div(V& vec, S m) { for (auto& val : vec) val /= m; return vec; }

			template <class V> V& conj(V& vec) { for (auto& val : vec) val = std::conj(val); return vec; }

			template <class V> V& sin(V& vec) { return elementwise(vec, ralgo::sin<value_t<V>>); }
			template <class V> V& cos(V& vec) { return elementwise(vec, ralgo::cos<value_t<V>>); }
			template <class V> V& tan(V& vec) { return elementwise(vec, ralgo::tan<value_t<V>>); }
			template <class V> V& exp(V& vec) { return elementwise(vec, ralgo::exp<value_t<V>>); }
			template <class V> V& log(V& vec) { return elementwise(vec, ralgo::log<value_t<V>>); }
			template <class V> V& log2(V& vec) { return elementwise(vec, ralgo::log2<value_t<V>>); }
			template <class V> V& log10(V& vec) { return elementwise(vec, ralgo::log10<value_t<V>>); }

			template <class V> V& normalize(V& vec)
			{
				double norm = ralgo::vecops::norm(vec);
				ralgo::vecops::inplace::div(vec, norm);
			}
		}

		template <class VI, class WI, class RI>
		void merge_sorted(VI vit, const VI vend, WI wit, const WI wend, RI rit)
		{
			while (vit != vend && wit != wend)
			{
				bool cmp = *vit < *wit;

				if (cmp) { *rit++ = *vit++; }
				else     { *rit++ = *wit++; }
			}

			while (vit != vend) { *rit++ = *vit++; }
			while (wit != wend) { *rit++ = *wit++; }
		}

		template <class V, class W, class R>
		void merge_sorted(const V& v, const W& w, R writer, double start, double stop)
		{
			auto vit = std::lower_bound(v.begin(), v.end(), start);
			auto wit = std::lower_bound(w.begin(), w.end(), start);
			auto vend = std::upper_bound(v.begin(), v.end(), stop);
			auto wend = std::upper_bound(w.begin(), w.end(), stop);

			return merge_sorted(vit, vend, wit, wend, writer);
		}

		template <class V, class W>
		auto merge_sorted(const V& v, const W& w, double start, double stop)
		{
			std::vector<value_t<V>> r;
			merge_sorted(v, w, std::back_inserter(r), start, stop);
			return r;
		}
	}

	/*using vecops::elementwise;
	using vecops::elementwise2;
	using vecops::vectorize;
	using vecops::arange;
	using vecops::linspace;
	using vecops::merge_sorted;*/

	namespace inplace = vecops::inplace;
}

#endif