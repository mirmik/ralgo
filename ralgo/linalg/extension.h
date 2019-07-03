#ifndef RALGO_LINALG_EXTENSION_H
#define RALGO_LINALG_EXTENSION_H

#include <linalg-v3/linalg.h>

namespace linalg 
{
    template<class T, int N> mat<T,N,N> exponent(const mat<T,N,N>& a) 
    {
    	mat<T,N,N> lres{}, res = identity, apow = a;
    	int i = 1;
    	res += a; 

    	while (lres != res) 
    	{
    		lres = res;
    		apow = (apow * a) / (++i);
    		res += apow;
    	}

    	return res;
    }

    namespace ostream_overloads
    {
        // These overloads stream out something that resembles an aggregate literal that could be used to construct the specified value
        template<class C, class T> std::basic_ostream<C> & operator << (std::basic_ostream<C> & out, const vec<T, 5> & v) { return out << '{' << v[0] << ',' << v[1] << ',' << v[2] << ',' << v[3] << ',' << v[4] << ',' << '}'; }
        template<class C, class T> std::basic_ostream<C> & operator << (std::basic_ostream<C> & out, const vec<T, 6> & v) { return out << '{' << v[0] << ',' << v[1] << ',' << v[2] << ',' << v[3] << ',' << v[4] << ',' << v[5] << '}'; }
    }
}

#endif