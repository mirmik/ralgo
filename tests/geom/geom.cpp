#include <doctest/doctest.h>
//#include <ralgo/geom/algtype.h>

/*TEST_CASE("SB")
{
    ralgo::geom::SB<double> a(1, {0, 0, 0});
    ralgo::geom::SB<double> b(2, {0, 1, 0});
    ralgo::geom::SB<double> c(2, {0, 1, 0});
    ralgo::geom::SB<double> d(0, {1, 0, 0});
    ralgo::geom::SB<double> f(0, {0, 1, 0});

    CHECK_EQ(geommul(a, b), ralgo::geom::SB<double>{2, {0, 1, 0}});
    CHECK_EQ(geommul(b, c), ralgo::geom::SB<double>{3, {0, 4, 0}});
    CHECK_EQ(geommul(d, f), ralgo::geom::SB<double>{0, {0, 0, -1}});
}

TEST_CASE("SB.rotor")
{
    auto rotor = ralgo::geom::rotor<double>({0, 0, M_PI});
    CHECK_EQ(rotor.e, doctest::Approx(0));
    CHECK_EQ(rotor.e23, doctest::Approx(0));
    CHECK_EQ(rotor.e31, doctest::Approx(0));
    CHECK_EQ(rotor.e12, doctest::Approx(1));

    auto unrotor = ralgo::geom::unrotor<double>(rotor);
    CHECK_EQ(unrotor.e23, doctest::Approx(0));
    CHECK_EQ(unrotor.e31, doctest::Approx(0));
    CHECK_EQ(unrotor.e12, doctest::Approx(M_PI));
}*/
