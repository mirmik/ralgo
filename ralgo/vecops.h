#ifndef RALGO_VECTOR_OPERATIONS_H
#define RALGO_VECTOR_OPERATIONS_H

#include <assert.h>
#include <algorithm>
#include <iterator>
#include <ralgo/util/helpers.h>
#include <igris/dprint.h>

namespace ralgo
{
	namespace vecops
	{
		// построить целочисленный вектор арифметической прогрессии [start;stop) с шагом step. 
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

		// Построить линейное пространство аля numpy.
		template<class R>
		auto linspace(double s, double f, int n)
		{
			R r(n);
			double k;

			for (int i = 0; i < n; ++i) 
			{
				k = (double)i / ((double)n - 1);
				r[i] = s * (1-k) + f * k;
			}

			return r;
		}

		// Применить функцию f ко всем элементам массива a. Допускается передача дополнительных аргументов.
		template <class R=void, class F, class A, class ... Args>
		defsame_t<R,A> elementwise(const F& f, const A & a, Args&& ... args)
		{
			defsame_t<R,A> ret(a.size());

			auto ait = a.begin(), aend = a.end();
			auto cit = ret.begin();

			for (; ait != aend; ++ait, ++cit)
				*cit = f(*ait, std::forward<Args>(args) ...);

			return ret;
		}

		// Применить функцию f ко всем элементам массивов a и b. Допускается передача дополнительных аргументов.
		template <class R, class F, class A, class B, class ... Args>
		R elementwise2(const F& f, const A& a, const B& b, Args&& ... args) 
		{
			assert(a.size() == b.size());
			R c(a.size());

			auto ait = a.begin(), aend = a.end();
			auto bit = b.begin();
			auto cit = c.begin();

			for (; ait != aend; ++ait, ++bit, ++cit)
				*cit = f(*ait, *bit, std::forward<Args>(args) ...); 

			return c;
		}

		template <class C, class F, class A, class B, class ... Args>
		void elementwise2_to(C& c, const F& f, const A& a, const B& b, Args&& ... args) 
		{
			auto ait = a.begin(), aend = a.end();
			auto bit = b.begin();
			auto cit = c.begin();

			for (; ait != aend; ++ait, ++bit, ++cit)
				*cit = f(*ait, *bit, std::forward<Args>(args) ...); 
		}


		// Создаёт векторизованный вриант функции.
		template<class V, class R = V, typename F, class ... Args>
		constexpr auto vectorize(const F& f, Args && ... args)
		{
			auto lamda = [=](const V & vec){ return elementwise<R>(f, vec, args...); };
			return lamda;
		}

		// Векторизованные операции над комплексными числами.
		template <class R=void, class A> defsame_t<R,A> abs(const A& obj) { return elementwise<R>(ralgo::op::abs(), obj); }
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

			for (auto& val : vec) { auto a = std::abs(val); res += a * a; }

			return sqrt(res);
		}

		// Вычислить расстояние между кортежами равного размера по 
		// евклидовой метрике.
		template <class R=double, class A, class B>
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
		template <class R=double, class A, class B>
		R distance(const A& a, const B& b) 
		{
			return sqrt(distance2<R,A,B>(a,b));
		}

		template <class R, class A>
		R cast(const A& a) 
		{
			return elementwise<R>([](auto x){ return x; }, a);
		}

		template <class R=void, class A, class B> defsame_t<R,A> add_vs(const A& a, B b) { return elementwise<R>(ralgo::op::add(), a, b); }
		template <class R=void, class A, class B> defsame_t<R,A> sub_vs(const A& a, B b) { return elementwise<R>(ralgo::op::sub(), a, b); }
		template <class R=void, class A, class B> defsame_t<R,A> mul_vs(const A& a, B b) { return elementwise<R>(ralgo::op::mul(), a, b); }
		template <class R=void, class A, class B> defsame_t<R,A> div_vs(const A& a, B b) { return elementwise<R>(ralgo::op::div(), a, b); }

		template <class R=void, class A, class B> defsame_t<R,A> add_vv(const A& a, const B& b) { return elementwise2<R>(ralgo::op::add(), a, b); }
		template <class R=void, class A, class B> defsame_t<R,A> sub_vv(const A& a, const B& b) { return elementwise2<R>(ralgo::op::sub(), a, b); }
		template <class R=void, class A, class B> defsame_t<R,A> mul_vv(const A& a, const B& b) { return elementwise2<R>(ralgo::op::mul(), a, b); }
		template <class R=void, class A, class B> defsame_t<R,A> div_vv(const A& a, const B& b) { return elementwise2<R>(ralgo::op::div(), a, b); }

		template <class A, class B, class C> void add_vv_to(C& c, const A& a, const B& b) { return elementwise2_to(c, ralgo::op::add(), a, b); }
		template <class A, class B, class C> void sub_vv_to(C& c, const A& a, const B& b) { return elementwise2_to(c, ralgo::op::sub(), a, b); }
		template <class A, class B, class C> void mul_vv_to(C& c, const A& a, const B& b) { return elementwise2_to(c, ralgo::op::mul(), a, b); }
		template <class A, class B, class C> void div_vv_to(C& c, const A& a, const B& b) { return elementwise2_to(c, ralgo::op::div(), a, b); }

		namespace inplace
		{
			// Применить функцию f ко всем элементам массива v. Допускается передача дополнительных аргументов.
			template <class V, class F, class ... A>
			V& elementwise(V& v, F f, A&& ... a)
			{
				for (auto & s : v) s = f(s, std::forward<A>(a) ...);
				return v;
			}

			template <class V> void clean(V& v) { 
				elementwise(v, [](auto& a){return typename V::value_type();}); }

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
			merge_sorted(v,w,std::back_inserter(r),start,stop);	
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