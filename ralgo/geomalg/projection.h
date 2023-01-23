#ifndef RALGO_GEOMALG_PROJECTION_H
#define RALGO_GEOMALG_PROJECTION_H

#include <ralgo/geomalg/line.h>
#include <ralgo/geomalg/plane.h>
#include <ralgo/geomalg/vector4d.h>
#include <ralgo/linalg/linalg.h>

namespace ralgo
{
    namespace geomalg
    {
        template <class T>
        ralgo::geomalg::point<T>
        projection_of_point_onto_line(const ralgo::geomalg::point<T> &p,
                                      const ralgo::geomalg::line<T> &l)
        {
            T scalar = l.vx() * p.x() + l.vy() * p.y() + l.vz() * p.z();
            ralgo::geomalg::point<T> pnt = {
                (l.vy() * l.mz() - l.vz() * l.my()) * p.w() + scalar * l.vx(),
                (l.vz() * l.mx() - l.vx() * l.mz()) * p.w() + scalar * l.vy(),
                (l.vx() * l.my() - l.vy() * l.mx()) * p.w() + scalar * l.vz(),
                (l.vx() * l.vx() + l.vy() * l.vy() + l.vz() * l.vz()) * p.w()};
            return pnt;
        }
    }
}

#endif