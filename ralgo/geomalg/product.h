#ifndef RALGO_GEOMALG_PRODUCT_H
#define RALGO_GEOMALG_PRODUCT_H

#include <ralgo/geomalg/complement.h>

namespace ralgo
{
    namespace geomalg
    {
        template <class T>
        bivector4d<T> wedge4d(const vector4d<T> &p, const vector4d<T> &q)
        {
            return bivector4d(p.w() * q.x() - p.x() * q.w(),
                              p.w() * q.y() - p.y() * q.w(),
                              p.w() * q.z() - p.z() * q.w(),
                              p.y() * q.z() - p.z() * q.y(),
                              p.z() * q.x() - p.x() * q.z(),
                              p.x() * q.y() - p.y() * q.x());
        }

        template <class T>
        bivector4d<T> wedge4d(const point3d<T> &p, const point3d<T> &q)
        {
            return bivector4d(q.x() - p.x(),
                              q.y() - p.y(),
                              q.z() - p.z(),
                              p.y() * q.z() - p.z() * q.y(),
                              p.z() * q.x() - p.x() * q.z(),
                              p.x() * q.y() - p.y() * q.x());
        }

        template <class T>
        bivector4d<T> wedge4d(const point3d<T> &p, const vector3d<T> &v)
        {
            return bivector4d(v.x(),
                              v.y(),
                              v.z(),
                              p.y() * v.z() - p.z() * v.y(),
                              p.z() * v.x() - p.x() * v.z(),
                              p.x() * v.y() - p.y() * v.x());
        }

        template <class T>
        trivector4d<T> wedge4d(const bivector4d<T> &l, const vector4d<T> &p)
        {
            double x = l.vy() * p.z() - l.vz() * p.y() + l.mx() * p.w();
            double y = l.vz() * p.x() - l.vx() * p.z() + l.my() * p.w();
            double z = l.vx() * p.y() - l.vy() * p.x() + l.mz() * p.w();
            double w = -(l.mx() * p.x() + l.my() * p.y() + l.mz() * p.z());
            return trivector4d(x, y, z, w);
        }

        template <class T> T dot(const vector2d<T> &a, const vector2d<T> &b)
        {
            return a.x() * b.x() + a.y() * b.y();
        }

        template <class T> T dot(const vector3d<T> &a, const vector3d<T> &b)
        {
            return a.x() * b.x() + a.y() * b.y() + a.z() * b.z();
        }
    }
}

#endif