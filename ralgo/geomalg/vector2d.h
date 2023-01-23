#ifndef RALGO_GEOMALG_VECTOR2D_H
#define RALGO_GEOMALG_VECTOR2D_H

#include <ralgo/linalg/linalg.h>

namespace ralgo
{
    namespace geomalg
    {
        template <class T> class vector2d
        {
            T _x;
            T _y;

        public:
            vector2d() = default;
            vector2d(T x, T y) : _x(x), _y(y) {}
            vector2d(const vector2d &v) = default;
            vector2d &operator=(const vector2d &v) = default;

            T x() const
            {
                return _x;
            }

            T y() const
            {
                return _y;
            }
        };
    }
}

#endif