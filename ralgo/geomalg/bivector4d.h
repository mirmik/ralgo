#ifndef RALGO_GEOMALG_BIVECTOR4D_H
#define RALGO_GEOMALG_BIVECTOR4D_H

#include <ralgo/geomalg/bivector3d.h>
#include <ralgo/geomalg/vector3d.h>

namespace ralgo
{
    namespace geomalg
    {
        template <class T> class bivector4d
        {
            vector3d<T> v;
            bivector3d<T> m;

        public:
            bivector4d() = default;
            bivector4d(linalg::vec<T, 3> v, linalg::vec<T, 3> m) : v(v), m(m) {}
            bivector4d(vector3d<T> v, bivector3d<T> m) : v(v), m(m) {}
            bivector4d(T vx, T vy, T vz, T mx, T my, T mz)
                : v(vx, vy, vz), m(mx, my, mz)
            {
            }

            bivector4d(const bivector4d &other) = default;
            bivector4d &operator=(const bivector4d &other) = default;

            const vector3d<T> &direction() const
            {
                return v;
            }

            const bivector3d<T> &momentum() const
            {
                return m;
            }

            T vx() const
            {
                return v.x();
            }

            T vy() const
            {
                return v.y();
            }

            T vz() const
            {
                return v.z();
            }

            T mx() const
            {
                return m.x();
            }

            T my() const
            {
                return m.y();
            }

            T mz() const
            {
                return m.z();
            }

            const vector3d<T> &bulk() const
            {
                return direction();
            }

            const bivector3d<T> &weight() const
            {
                return momentum();
            }

            bivector4d unitized() const
            {
                T n = v.length();
                return bivector4d(v / n, m / n);
            }
        };
    }
}

#endif