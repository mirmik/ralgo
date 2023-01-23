#ifndef RALGO_GEOMALG_LINE_H
#define RALGO_GEOMALG_LINE_H

#include <ralgo/geomalg/point.h>

namespace ralgo
{
    namespace geomalg
    {
        template <class T> class line
        {
        private:
            linalg::vec<T, 3> _direction;
            linalg::vec<T, 3> _momentum;

        public:
            line(const linalg::vec<T, 3> &direction,
                 const linalg::vec<T, 3> &momentum)
                : _direction(direction), _momentum(momentum)
            {
            }

            T vx() const
            {
                return _direction[0];
            }

            T vy() const
            {
                return _direction[1];
            }

            T vz() const
            {
                return _direction[2];
            }

            T mx() const
            {
                return _momentum[0];
            }

            T my() const
            {
                return _momentum[1];
            }

            T mz() const
            {
                return _momentum[2];
            }

            static line<T> from_points(const point<T> &p, const point<T> &q)
            {
                linalg::vec<T, 3> direction(p.w() * q.x() - p.x() * q.w(),
                                            p.w() * q.y() - p.y() * q.w(),
                                            p.w() * q.z() - p.z() * q.w());
                linalg::vec<T, 3> momentum(p.y() * q.z() - p.z() * q.y(),
                                           p.z() * q.x() - p.x() * q.z(),
                                           p.x() * q.y() - p.y() * q.x());
                return line<T>(direction, momentum);
            }

            const linalg::vec<T, 3> &direction() const
            {
                return _direction;
            }

            const linalg::vec<T, 3> &momentum() const
            {
                return _momentum;
            }

            line unitized() const
            {
                auto n = linalg::length(_direction);
                return line(_direction / n, _momentum / n);
            }
        };
    }

} // namespace ralgo

#endif