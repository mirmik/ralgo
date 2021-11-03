#include <doctest/doctest.h>
#include <ralgo/lp/gradient.h>

TEST_CASE("convex_gradient_descent")
{
    int dim = 2;
    int points = 3;
    double alpha = 0.05;
    double epsilon = 1e-5;

    double table[] = {0, 0, 2, 0, 0, 2};

    double result[] = {0, 0, 0};
    double target0[] = {1, 0};
    double target1[] = {0.9, 0.9};

    convex_gradient_descent(dim, points, alpha, epsilon,
                            table,   // size: dim * points
                            target0, // size: dim
                            result   // size: dim
    );

    CHECK_EQ(result[0], 0);
    CHECK_EQ(result[1], doctest::Approx(0.5));
    CHECK_EQ(result[2], 0);

    convex_gradient_descent(dim, points, alpha, epsilon,
                            table,   // size: dim * points
                            target1, // size: dim
                            result   // size: dim
    );

    CHECK_EQ(result[0], 0);
    CHECK_EQ(result[1], doctest::Approx(0.45));
    CHECK_EQ(result[2], doctest::Approx(0.45));
}
