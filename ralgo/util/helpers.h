#ifndef RALGO_UTIL_FUNC_HELPERS_H
#define RALGO_UTIL_FUNC_HELPERS_H

#include <cmath>
#include <numeric>
#include <complex>
#include <vector>

namespace ralgo 
{
	constexpr const double epsilon = std::numeric_limits<double>::epsilon();

	// Достать тип значения из контейнера.
	template <typename V> struct value { using type = typename V::value_type; };
	template <typename T, int N> struct value<T[N]> { using type = T; };
	template <typename V> using value_t = typename value<V>::type;

	// Превести тип комплексного аргумента или вектора к соответствующему базовому скалярному типу.
	template <typename T> struct scalar { using type = T; };
	template <typename T> struct scalar<std::complex<T>> { using type = T; };
	template <typename T> using scalar_t = typename scalar<T>::type;

	// Шаблонные обёртки над стандартными тригонометрическими операциями.
	template <class T> constexpr T sin(T x) { return std::sin(x); }
	template <class T> constexpr T cos(T x) { return std::cos(x); }
	template <class T> constexpr T tan(T x) { return std::tan(x); }

	template <class T> constexpr T exp(T x) { return std::exp(x); }
	template <class T> constexpr T log(T x) { return std::log(x); }
	template <class T> constexpr T log2(T x) { return std::log2(x); }
	template <class T> constexpr T log10(T x) { return std::log10(x); }

		struct op_add { template<class A, class B> auto operator()(const A& a, const B& b) const { return a + b; } };
		struct op_sub { template<class A, class B> auto operator()(const A& a, const B& b) const { return a - b; } };
		struct op_mul { template<class A, class B> auto operator()(const A& a, const B& b) const { return a * b; } };
		struct op_div { template<class A, class B> auto operator()(const A& a, const B& b) const { return a / b; } };
	
		struct op_abs { template<class A> auto operator()(const A& a) const { return std::abs(a); } };

		struct op_eq { template<class A, class B> auto operator()(const A& a, const B& b) const { return a == b; } };
		struct op_not_eq { template<class A, class B> auto operator()(const A& a, const B& b) const { return a != b; } };

		struct op_and { template<class A, class B> bool operator()(const A& a, const B& b) const { return a && b; } };
		struct op_or { template<class A, class B> bool operator()(const A& a, const B& b) const { return a || b; } };
		struct op_bin_and { template<class A, class B> auto operator()(const A& a, const B& b) const { return a & b; } };
		struct op_bin_or { template<class A, class B> auto operator()(const A& a, const B& b) const { return a | b; } };

	template <typename ... T> struct rettype {};
	template <class A, class B> struct rettype<A,B> { using type = A; };
///	template <template<class> V, class T> struct rettype { using type = V<T>; };
	template <typename ... T> using rettype_t = typename rettype<T ...>::type;

	// Определяет тип по умолчанию. Если тип равен void, то используется std::vector<>.
	template<class R, class V> struct defvec { using type = R; };
	template<class V> struct defvec<void, V> { using type = std::vector<value_t<V>>; };
	template<class R, class V> using defvec_t = typename defvec<R,V>::type; 

	// Определяет тип по умолчанию. Если тип равен void, то используется std::vector<>.
	template<class R, class V> struct defvec_of { using type = R; };
	template<class V> struct defvec_of<void, V> { using type = std::vector<V>; };
	template<class R, class V> using defvec_of_t = typename defvec_of<R,V>::type; 

	// Определяет тип по умолчанию. Если тип равен void, то используется тип результат функтора.
	template<class R, class F> struct fretvec { using type = R; };
	template<class F> struct fretvec<void, F> { using type = std::vector<std::result_of<F>>; };
	template<class R, class F> using fretvec_t = typename fretvec<R,F>::type; 

	// Определяет тип по умолчанию. Если тип равен void, то используется тип по умолчанию.
	template<class R, class V> struct defsame { using type = R; };
	template<class V> struct defsame<void, V> { using type = V; };
	template<class R, class V> using defsame_t = typename defsame<R, V>::type;

}

#endif