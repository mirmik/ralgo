#ifndef RALGO_GEOMALG_DISTANCE_H
#define RALGO_GEOMALG_DISTANCE_H

#include <ralgo/geomalg/magnitude.h>
#include <ralgo/geomalg/vector4d.h>

namespace ralgo
{
    namespace geomalg
    {
        template <class T>
        magnitude<T> distance_between_points(const vector4d<T> &p,
                                             const vector4d<T> &q)
        {
            auto x = q.x() * p.w() - p.x() * q.w();
            auto y = q.y() * p.w() - p.y() * q.w();
            auto z = q.z() * p.w() - p.z() * q.w();
            auto s = std::sqrt(x * x + y * y + z * z);
            auto w = p.w() * q.w();
            return {s, w};
        }
    }
}

#endif