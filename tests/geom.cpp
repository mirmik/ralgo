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