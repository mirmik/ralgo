#ifndef RALGO_GEOM3_RAY_H
#define RALGO_GEOM3_RAY_H

namespace ralgo
{
    namespace geom3
    {
        template <class T> class ray
        {
        private:
            linalg::vec<T, 3> _direction;
            linalg::vec<T, 3> _center;

        public:
            ray(const linalg::vec<T, 3> &direction,
                const linalg::vec<T, 3> &point)
                : _direction(direction),
                  _center(linalg::cross(direction, point))
            {
            }

            linalg::vec<T, 3> direction() const
            {
                return _direction;
            }

            linalg::vec<T, 3> center() const
            {
                return _center;
            }
        };
    }
}