#ifndef RALGO_VECOPS_BASE_H
#define RALGO_VECOPS_BASE_H

#include <type_traits>
#include <ralgo/util/helpers.h>

namespace ralgo
{
	namespace vecops
	{

		/*		// Тип, соответствующий типу возвращаемому при итерации контейнера.
				template <class V> using veciter_ret_t = std::result_of<decltype(V::operator[])>;

				// Итератор, передаваемый в обобщенные функции, использующие распаковку.
				template <class A> struct veciter
				{
					const A& ref;
					constexpr inline veciter(const A& ref) : ref(ref) {}
				};

				// Функция итерирует
				template <class A> struct veciter_unpack
				{
					constexpr auto operator()(A&& arg)
					{
						return std::forward<A>(arg);
					}
				};

				template <class A> struct iter_unpack<veciter<A>>
				{
					constexpr auto operator()(const veciter<A>& arg)
					{
						return arg.ref;
					}
				};*/

		// Свёртка списка по функции func.
		template <class F, class A, class R = std::result_of_t<F(A)>>
		R fold(F && func, const R& initval, const A& a)
		{
			R accum = initval;

			auto ait = a.begin();
			auto aeit = a.end();

			for (; ait != aeit; ++ait)
			{ accum = func(accum, *ait); }

			return accum;
		}

		template <class F, class A, class B, class ... Args> bool boolean_all(F&& func, const A& a, const B& b, Args && ... args) { auto ait = a.begin(); auto bit = b.begin(); auto aeit = a.end(); for (; ait != aeit; ++ait, ++bit) if (func(*ait, *bit, std::forward<Args>(args) ... ) == false) return false; return true;  }
		template <class F, class A, class B, class ... Args> bool boolean_any(F&& func, const A& a, const B& b, Args && ... args) { auto ait = a.begin(); auto bit = b.begin(); auto aeit = a.end(); for (; ait != aeit; ++ait, ++bit) if (func(*ait, *bit, std::forward<Args>(args) ... ) == true)  return true;  return false; }

		// Применить функцию f ко всем элементам массива a. Допускается передача дополнительных аргументов.
		template <class R = void, class F, class A, class ... Args>
		defsame_t<R, A> elementwise(const F& f, const A & a, Args && ... args)
		{
			defsame_t<R, A> ret(a.size());

			auto ait = a.begin(), aend = a.end();
			auto cit = ret.begin();

			for (; ait != aend; ++ait, ++cit)
				*cit = f(*ait, std::forward<Args>(args) ...);

			return ret;
		}

		// Применить функцию f ко всем элементам массивов a и b. Допускается передача дополнительных аргументов.
		template <class R = void, class F, class A, class B, class ... Args>
		defsame_t<R, A> elementwise2(const F& f, const A& a, const B& b, Args && ... args)
		{
			assert(a.size() == b.size());
			defsame_t<R, A> c(a.size());

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

	}
}

#endif