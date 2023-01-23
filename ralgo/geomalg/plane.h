#ifndef RALGO_GEOMALG_PLANE_H
#define RALGO_GEOMALG_PLANE_H

#include <ralgo/linalg/linalg.h>

namespace ralgo
{
    namespace geomalg
    {
        template <class T> class plane
        {
        private:
            linalg::vec<T, 3> _normal;
            T _offset;

        public:
            T x() const
            {
                return _normal[0];
            }

            T y() const
            {
                return _normal[1];
            }

            T z() const
            {
                return _normal[2];
            }

            T w() const
            {
                return _offset;
            }

            plane(T x, T y, T z, T w) : _normal(x, y, z), _offset(w) {}

            plane(const linalg::vec<T, 3> &normal, T offset)
                : _normal(normal), _offset(offset)
            {
            }

            plane(const linalg::vec<T, 3> &normal,
                  const linalg::vec<T, 3> &point)
                : _normal(normal), _offset(linalg::dot(normal, point))
            {
            }

            plane(const linalg::vec<T, 3> &a,
                  const linalg::vec<T, 3> &b,
                  const linalg::vec<T, 3> &c)
                : _normal(linalg::cross(b - a, c - a)),
                  _offset(linalg::dot(_normal, a))
            {
            }

            plane(std::array<linalg::vec<T, 3>, 3> points)
                : plane(points[0], points[1], points[2])
            {
            }

            linalg::vec<T, 3> normal() const
            {
                return _normal;
            }
            T offset() const
            {
                return _offset;
            }

            T distance(const linalg::vec<T, 3> &point) const
            {
                return linalg::dot(_normal, point) - _offset;
            }

            linalg::vec<T, 3> project(const linalg::vec<T, 3> &point) const
            {
                return point - _normal * distance(point);
            }

            plane unitized() const
            {
                T len = linalg::length(_normal);
                return plane(_normal / len, _offset / len);
            }
        };
    }
}

#endif