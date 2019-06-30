#ifndef RALGO_UTIL_FUNC_HELPERS_H
#define RALGO_UTIL_FUNC_HELPERS_H

#include <cmath>

namespace ralgo 
{
	// Достать тип значения из контейнера.
	template <typename V> struct value { using type = typename V::value_type; };
	template <typename T> using value_t = typename value<T>::type;

	// Превести тип комплексного аргумента или вектора к соответствующему базовому скалярному типу.
	template <typename T> struct scalar { using type = T; };
	template <typename T> struct scalar<std::complex<T>> { using type = T; };
	template <typename T> using scalar_t = typename scalar<T>::type;

	// Шаблонные обёртки над стандартными тригонометрическими операциями.
	template <class T> constexpr T sin(T x) { return std::sin(x); }
	template <class T> constexpr T cos(T x) { return std::cos(x); }
	template <class T> constexpr T tan(T x) { return std::tan(x); }
}

#endif