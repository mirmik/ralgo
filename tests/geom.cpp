#include <ralgo/geom/algtype.h>
#include <doctest/doctest.h>

TEST_CASE("scalar_bivector") 
{
    ralgo::geom::scalar_bivector<double> a(1, {0, 0, 0});
    ralgo::geom::scalar_bivector<double> b(2, {0, 1, 0});
    ralgo::geom::scalar_bivector<double> c(2, {0, 1, 0});
    ralgo::geom::scalar_bivector<double> d(0, {1, 0, 0});
    ralgo::geom::scalar_bivector<double> f(0, {0, 1, 0});
    
    CHECK_EQ(geommul(a, b), ralgo::geom::scalar_bivector<double>{2, {0, 1, 0}});
    CHECK_EQ(geommul(b, c), ralgo::geom::scalar_bivector<double>{3, {0, 4, 0}});
    CHECK_EQ(geommul(d, f), ralgo::geom::scalar_bivector<double>{0, {0, 0, -1}});
}


TEST_CASE("scalar_bivector.rotor") 
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
} 