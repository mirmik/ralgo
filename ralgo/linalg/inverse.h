#ifndef RALGO_LINALG_INVERSE_H
#define RALGO_LINALG_INVERSE_H

#include <ralgo/util/helpers.h>
#include <ralgo/linalg/matrix.h>
#include <ralgo/linalg/plud.h>

namespace ralgo 
{
    template <class R=void, class A>
    defsame_t<R, ralgo::matrix<typename A::value_type>> inverse(const A &a)
    {
        defsame_t<R, ralgo::matrix<typename A::value_type>> inv;
        auto plud = ralgo::plud(a);
        plud.inverse(inv);
        return inv;
    }
}

#endif