#ifndef RALGO_GEOMALG_PLANE_H
#define RALGO_GEOMALG_PLANE_H

#include <ralgo/geomalg/trivector4d.h>
#include <ralgo/linalg/linalg.h>

namespace ralgo
{
    namespace geomalg
    {
        template <class T> class plane : public trivector4d<T>
        {
        public:
            plane() = default;
            plane(T x, T y, T z, T w) : trivector4d<T>(x, y, z, w) {}
            plane(const trivector4d<T> &v) : trivector4d<T>(v) {}
            plane(const plane &v) = default;
            plane &operator=(const plane &v) = default;

            // construct plane from normal and point
            // normal must be unitized
            plane(const vector3d<T> &n, const point3d<T> &p)
                : trivector4d<T>(n, -dot(n, p))
            {
            }

            plane unitized() const
            {
                return plane(trivector4d<T>::unitized());
            }
        };
    }
}

#endif