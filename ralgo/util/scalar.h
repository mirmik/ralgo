#ifndef RALGO_SCALAR_H
#define RALGO_SCALAR_H

#include <complex>

namespace ralgo 
{
	template <typename T> struct scalar { using type = T; };
	template <typename T> struct scalar<std::complex<T>> { using type = T; };

	template <typename T> using scalar_t = typename scalar<T>::type;
}

#endif