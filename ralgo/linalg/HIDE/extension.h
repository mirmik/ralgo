#ifndef RALGO_LINALG_EXTENSION_H
#define RALGO_LINALG_EXTENSION_H

#include <linalg/linalg.h>

namespace linalg
{
    template<class T, int N> mat<T, N, N> exponent(const mat<T, N, N>& a)
    {
        mat<T, N, N> lres{}, res = identity, apow = a;
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

    template<class T, int M> T pseudolen0 (const vec<T, M> & a) { return max(abs(a)); }
    template<class T, int M> T pseudolen1 (const vec<T, M> & a) { return sum(abs(a)); }
}

#endif