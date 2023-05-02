#include <doctest/doctest.h>
#include <ralgo/boundary_box.h>

TEST_CASE("boundary_box")
{
    ralgo::boundary_box<double> box({1, 2, 3}, {3, 4, 5});

    ralgo::vector<double> pnt({2, 3, 4});

    auto coeffs = box.lerpcoeffs(pnt);

    CHECK_EQ(coeffs[0], 0.5);
    CHECK_EQ(coeffs[1], 0.5);
    CHECK_EQ(coeffs[2], 0.5);
}