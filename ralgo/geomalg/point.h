#ifndef RALGO_GEOMALG_POINT_H
#define RALGO_GEOMALG_POINT_H

namespace ralgo
{
    namespace geomalg
    {
        template <class T> class point
        {
            linalg::vec<T, 3> _vec;
            double _w;

        public:
            T x() const
            {
                return _vec[0];
            }

            T y() const
            {
                return _vec[1];
            }

            T z() const
            {
                return _vec[2];
            }

            T w() const
            {
                return _w;
            }

            point(T x, T y, T z, T w) : _vec(x, y, z), _w(w) {}
            point(T x, T y, T z) : _vec(x, y, z), _w(1) {}

            point(const linalg::vec<T, 3> &vec, T w) : _vec(vec), _w(w) {}

            point(const linalg::vec<T, 3> &vec) : _vec(vec), _w(1) {}

            point unitized() const
            {
                return point(_vec / _w, 1);
            }
        };
    }

} // namespace ralgo

#endif