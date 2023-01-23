#ifndef RALGO_GEOMALG_OPS_H
#define RALGO_GEOMALG_OPS_H

#include <ralgo/geomalg/line.h>
#include <ralgo/geomalg/plane.h>
#include <ralgo/geomalg/point.h>

namespace ralgo
{
    namespace geomalg
    {

        template <class T>
        ralgo::geomalg::line<T>
        line_containing_points(const ralgo::geomalg::point<T> &p,
                               const ralgo::geomalg::point<T> &q)
        {
            linalg::vec<T, 3> v(p.w() * q.x() - p.x() * q.w(),
                                p.w() * q.y() - p.y() * q.w(),
                                p.w() * q.z() - p.z() * q.w());
            linalg::vec<T, 3> m(p.y() * q.z() - p.z() * q.y(),
                                p.z() * q.x() - p.x() * q.z(),
                                p.x() * q.y() - p.y() * q.x());
            return ralgo::geomalg::line(v, m);
        }

        template <class T>
        ralgo::geomalg::plane<T>
        plane_containing_line_and_point(const ralgo::geomalg::line<T> &l,
                                        const ralgo::geomalg::point<T> &p)
        {
            double x = l.vy() * p.z() - l.vz() * p.y() + l.mx() * p.w();
            double y = l.vz() * p.x() - l.vx() * p.z() + l.my() * p.w();
            double z = l.vx() * p.y() - l.vy() * p.x() + l.mz() * p.w();
            double w = -(l.mx() * p.x() + l.my() * p.y() + l.mz() * p.z());
            return ralgo::geomalg::plane(x, y, z, w);
        }

        template <class T>
        ralgo::geomalg::plane<T>
        plane_containing_points(const ralgo::geomalg::point<T> &a,
                                const ralgo::geomalg::point<T> &b,
                                const ralgo::geomalg::point<T> &c)
        {
            ralgo::geomalg::line<T> l = line_containing_points(a, b);
            return plane_containing_line_and_point(l, c);
        }

        template <class T>
        ralgo::geomalg::line<T>
        line_where_plane_intersects_plane(const ralgo::geomalg::plane<T> &g,
                                          const ralgo::geomalg::plane<T> &h)
        {
            linalg::vec<T, 3> v(g.z() * h.y() - g.y() * h.z(),
                                g.x() * h.z() - g.z() * h.x(),
                                g.y() * h.x() - g.x() * h.y());
            linalg::vec<T, 3> m(g.x() * h.w() - g.w() * h.x(),
                                g.y() * h.w() - g.w() * h.y(),
                                g.z() * h.w() - g.w() * h.z());
            return ralgo::geomalg::line(v, m);
        }

        template <class T>
        ralgo::geomalg::point<T>
        point_where_line_intersects_plane(const ralgo::geomalg::line<T> &l,
                                          const ralgo::geomalg::plane<T> &g)
        {
            double x = l.my() * g.z() - l.mz() * g.y() + l.vx() * g.w();
            double y = l.mz() * g.x() - l.mx() * g.z() + l.vy() * g.w();
            double z = l.mx() * g.y() - l.my() * g.x() + l.vz() * g.w();
            double w = -(l.vx() * g.x() + l.vy() * g.y() + l.vz() * g.z());
            return ralgo::geomalg::point(x, y, z, w);
        }
    }
}
#endif