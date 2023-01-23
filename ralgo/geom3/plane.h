#ifndef RALGO_GEOM3_PLANE_H
#define RALGO_GEOM3_PLANE_H

namespace ralgo
{
    namespace geom3
    {
        template <class T> class plane
        {
        private:
            linalg::vec<T, 3> _normal;
            T _offset;

        public:
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
        };
    }
}

#endif