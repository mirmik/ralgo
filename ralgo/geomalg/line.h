#ifndef RALGO_GEOMALG_LINE_H
#define RALGO_GEOMALG_LINE_H

#include <ralgo/geomalg/bivector4d.h>
#include <ralgo/geomalg/product.h>
#include <ralgo/geomalg/vector3d.h>
#include <ralgo/geomalg/vector4d.h>

namespace ralgo
{
    namespace geomalg
    {
        template <class T> class line : public ralgo::geomalg::bivector4d<T>
        {
        public:
            line(const linalg::vec<T, 3> &direction,
                 const linalg::vec<T, 3> &momentum)
                : bivector4d<T>(direction, momentum)
            {
            }

            line(const bivector4d<T> &bivector) : bivector4d<T>(bivector) {}

            static line<T> from_points(const vector4d<T> &p,
                                       const vector4d<T> &q)
            {
                return line<T>(wedge4d(p, q));
            }

            static line<T> from_points(const point3d<T> &p, const point3d<T> &q)
            {
                return line<T>(wedge4d(p, q));
            }

            line unitized() const
            {
                return line(bivector4d<T>::unitized());
            }
        };
    }

} // namespace ralgo

#endif