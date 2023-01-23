#include <doctest/doctest.h>
#include <ralgo/geom/algebra/complement.h>
#include <ralgo/geom/algebra/distance.h>
#include <ralgo/geom/algebra/join.h>
#include <ralgo/geom/algebra/meet.h>
#include <ralgo/geom/algebra/projection.h>
#include <ralgo/geom/algebra/types.h>
#include <ralgo/geom/algebra/types_minimal.h>
#include <ralgo/geom/product.h>

TEST_CASE("geomalg::vector")
{
    ralgo::geomalg::vector<double> a(1, 2, 3);
    ralgo::geomalg::vector<double> b(3, 4, 5);
    ralgo::geomalg::bivector<double> bv(8, 9, 10);
    CHECK_EQ(a.e1, 1);
    CHECK_EQ(a.e2, 2);
    CHECK_EQ(a.e3, 3);

    auto c = a + b;
    CHECK_EQ(c.e1, 4);
    CHECK_EQ(c.e2, 6);
    CHECK_EQ(c.e3, 8);

    auto d = a + bv;
    CHECK_EQ(d.e1, 1);
    CHECK_EQ(d.e2, 2);
    CHECK_EQ(d.e3, 3);
    CHECK_EQ(d.e23, 8);
    CHECK_EQ(d.e31, 9);
    CHECK_EQ(d.e12, 10);
}

TEST_CASE("line_point")
{
    ralgo::geomalg::point<double> p(0, 0, 1);
    ralgo::geomalg::point<double> q(1, 0, 1);
    ralgo::geomalg::point<double> p2(0, 0, 2);
    ralgo::geomalg::point<double> q2(1, 0, 2);
    ralgo::geomalg::point<double> q22(2, 0, 2);
    ralgo::geomalg::point<double> z(0, 0, 0);
    auto l = join(p, q);
    auto l3 = join(q, p);
    auto l2 = join(p2, q2);
    auto l22 = join(p2, q22);
    auto m = join(z, p);
    CHECK_EQ(l.direction(), linalg::vec<double, 3>{1, 0, 0});
    CHECK_EQ(l.momentum(), linalg::vec<double, 3>{0, 1, 0});
    CHECK_EQ(l3.direction(), linalg::vec<double, 3>{-1, 0, 0});
    CHECK_EQ(l3.momentum(), linalg::vec<double, 3>{0, -1, 0});
    CHECK_EQ(l2.direction(), linalg::vec<double, 3>{1, 0, 0});
    CHECK_EQ(l2.momentum(), linalg::vec<double, 3>{0, 2, 0});
    CHECK_EQ(l22.direction(), linalg::vec<double, 3>{2, 0, 0});
    CHECK_EQ(l22.momentum(), linalg::vec<double, 3>{0, 4, 0});
    CHECK_EQ(m.direction(), linalg::vec<double, 3>{0, 0, 1});
    CHECK_EQ(m.momentum(), linalg::vec<double, 3>{0, 0, 0});
}

TEST_CASE("line_point.check_scalar is zero")
{
    ralgo::geomalg::point<double> p(53, 22, 12);
    ralgo::geomalg::point<double> q(1, 11, 1);
    auto l = join(p, q);
    CHECK_EQ(dot(l.direction(), complement(l.momentum())), 0);
}

TEST_CASE("line_containing_points")
{
    ralgo::geomalg::point<double> p(53, 22, 12);
    ralgo::geomalg::point<double> q(1, 11, 1);
    auto l = ralgo::geomalg::line_containing_points(p, q);
    CHECK_EQ(dot(l.direction(), complement(l.momentum())), 0);
}

TEST_CASE("plane_containing_line_and_point")
{
    ralgo::geomalg::point<double> a(0, 1, 0);
    ralgo::geomalg::point<double> b(1, 0, 0);
    ralgo::geomalg::point<double> z(0, 0, 0);
    auto l = ralgo::geomalg::line_containing_points(a, z);
    auto pl = ralgo::geomalg::plane_containing_line_and_point(l, b);
    CHECK_EQ(pl.x(), 0);
    CHECK_EQ(pl.y(), 0);
    CHECK_EQ(pl.z(), 1);
    CHECK_EQ(pl.w(), 0);
}

TEST_CASE("plane_containing_line_and_point_2")
{
    ralgo::geomalg::point<double> a(0, 1, 1);
    ralgo::geomalg::point<double> b(1, 0, 1);
    ralgo::geomalg::point<double> z(0, 0, 1);
    auto l = ralgo::geomalg::line_containing_points(a, z);
    auto pl = ralgo::geomalg::plane_containing_line_and_point(l, b);
    CHECK_EQ(pl.x(), 0);
    CHECK_EQ(pl.y(), 0);
    CHECK_EQ(pl.z(), 1);
    CHECK_EQ(pl.w(), -1);
}

TEST_CASE("point_where_line_intersects_plane")
{
    ralgo::geomalg::point<double> a(0, 1, 1);
    ralgo::geomalg::point<double> b(1, 0, 1);
    ralgo::geomalg::point<double> z(0, 0, 1);
    auto l = ralgo::geomalg::line_containing_points(a, z);
    auto pl = ralgo::geomalg::plane_containing_line_and_point(l, b);

    ralgo::geomalg::point<double> p(0, 1, 2);
    ralgo::geomalg::point<double> q(0, 1, 3);
    auto l2 = ralgo::geomalg::line_containing_points(p, q);

    auto r =
        ralgo::geomalg::point_where_line_intersects_plane(l2, pl).unitized();
    CHECK_EQ(r.x(), 0);
    CHECK_EQ(r.y(), 1);
    CHECK_EQ(r.z(), 1);
    CHECK_EQ(r.w(), 1);
}

TEST_CASE("line_where_plane_intersects_plane")
{
    ralgo::geomalg::point<double> a(0, 2, 2);
    ralgo::geomalg::point<double> b(2, 0, 2);
    ralgo::geomalg::point<double> c(0, 0, 2);
    auto p0 = ralgo::geomalg::plane_containing_points(a, b, c).unitized();
    CHECK_EQ(p0.x(), 0);
    CHECK_EQ(p0.y(), 0);
    CHECK_EQ(p0.z(), -1);
    CHECK_EQ(p0.w(), 2);

    ralgo::geomalg::point<double> x(1, 0, 0);
    ralgo::geomalg::point<double> y(1, 1, 0);
    ralgo::geomalg::point<double> z(1, 0, 1);
    auto p1 = ralgo::geomalg::plane_containing_points(x, y, z);
    CHECK_EQ(p1.x(), 1);
    CHECK_EQ(p1.y(), 0);
    CHECK_EQ(p1.z(), 0);
    CHECK_EQ(p1.w(), -1);

    auto l = ralgo::geomalg::line_where_plane_intersects_plane(p0, p1);
    CHECK_EQ(l.direction(), linalg::vec<double, 3>{0, 1, 0});
    CHECK_EQ(l.momentum(), linalg::vec<double, 3>{-2, 0, 1});
}

TEST_CASE("projection_of_point_onto_line")
{
    ralgo::geomalg::point<double> a(3, 0, -2);
    ralgo::geomalg::point<double> b(0, 0, -2);
    ralgo::geomalg::point<double> z(10, 10, -2);

    auto l = ralgo::geomalg::line_containing_points(a, b);
    auto r = ralgo::geomalg::projection_of_point_onto_line(z, l).unitized();

    CHECK_EQ(r.x(), 10);
    CHECK_EQ(r.y(), 0);
    CHECK_EQ(r.z(), -2);
    CHECK_EQ(r.w(), 1);
}

TEST_CASE("distance_between_points")
{
    ralgo::geomalg::point<double> a(3, 0, -2);
    ralgo::geomalg::point<double> b(0, 0, -2);
    auto mag = ralgo::geomalg::distance_between_points(a, b);

    CHECK_EQ(mag.s(), 3);
    CHECK_EQ(mag.w(), 1);
}

TEST_CASE("distance_between_points_2")
{
    ralgo::geomalg::point<double> a(3, 0, -8);
    ralgo::geomalg::point<double> b(0, 0, -2, 1.0 / 4);
    auto mag = ralgo::geomalg::distance_between_points(a, b).unitized();

    CHECK_EQ(mag.s(), 3);
    CHECK_EQ(mag.w(), 1);
}

TEST_CASE("distance_between_point_and_line")
{
    ralgo::geomalg::point<double> a(1, 0, -2);
    ralgo::geomalg::point<double> b(0, 0, -2);
    ralgo::geomalg::point<double> d(0, 0, -8, 2);
    auto l = ralgo::geomalg::line_containing_points(a, b);
    auto mag = distance_between_point_and_line(d, l);

    CHECK_EQ(mag.s(), 4);
    CHECK_EQ(mag.w(), 2);

    auto umag = mag.unitized();
    CHECK_EQ(umag.s(), 2);
    CHECK_EQ(umag.w(), 1);
}
