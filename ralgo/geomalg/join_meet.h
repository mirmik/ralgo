#ifndef RALGO_GEOMALG_OPS_H
#define RALGO_GEOMALG_OPS_H

#include <ralgo/geomalg/line.h>
#include <ralgo/geomalg/plane.h>

namespace ralgo
{
    namespace geomalg
    {

        template <class T>
        ralgo::geomalg::line<T>
        line_containing_points(const ralgo::geomalg::point3d<T> &p,
                               const ralgo::geomalg::point3d<T> &q)
        {
            return wedge4d(p, q);
        }

        template <class T>
        ralgo::geomalg::line<T>
        line_containing_points(const ralgo::geomalg::vector4d<T> &p,
                               const ralgo::geomalg::vector4d<T> &q)
        {
            return wedge4d(p, q);
        }

        template <class T>
        ralgo::geomalg::plane<T>
        plane_containing_line_and_point(const ralgo::geomalg::line<T> &l,
                                        const ralgo::geomalg::vector4d<T> &p)
        {
            return wedge4d(l, p);
        }

        template <class T>
        ralgo::geomalg::plane<T>
        plane_containing_points(const ralgo::geomalg::point<T> &a,
                                const ralgo::geomalg::point<T> &b,
                                const ralgo::geomalg::point<T> &c)
        {
            return wedge4d(wedge4d(a, b), c);
        }

        template <class T>
        ralgo::geomalg::line<T>
        line_where_plane_intersects_plane(const ralgo::geomalg::plane<T> &g,
                                          const ralgo::geomalg::plane<T> &h)
        {
            return antiwedge4d(g, h);
        }

        template <class T>
        ralgo::geomalg::point<T>
        point_where_line_intersects_plane(const ralgo::geomalg::line<T> &l,
                                          const ralgo::geomalg::plane<T> &g)
        {
            return antiwedge4d(l, g);
        }
    }
}
#endif