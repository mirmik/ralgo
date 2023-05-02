#include <doctest/doctest.h>
#include <ralgo/number_line.h>

TEST_CASE("number_line")
{
    std::vector<double> bounds = {1, 2, 3};

    CHECK_EQ(ralgo::number_line_interval_for_point(bounds, 0.5), 0);
    CHECK_EQ(ralgo::number_line_interval_for_point(bounds, 1.5), 1);
    CHECK_EQ(ralgo::number_line_interval_for_point(bounds, 2.5), 2);
    CHECK_EQ(ralgo::number_line_interval_for_point(bounds, 3.5), 3);
}