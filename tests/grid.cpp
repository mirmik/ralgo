#include <doctest/doctest.h>
#include <ralgo/cartesian_grid.h>

TEST_CASE("cartesian_grid")
{
    ralgo::cartesian_grid<double> grid({{1, 2, 3}, {4, 5, 6}});

    {
        CHECK_EQ(grid.cell_for_point(ralgo::vector<double>({0.5, 3.5})),
                 ralgo::vector<double>({0, 0}));

        CHECK_EQ(grid.cell_for_point(ralgo::vector<double>({1.5, 3.5})),
                 ralgo::vector<double>({1, 0}));

        CHECK_EQ(grid.cell_for_point(ralgo::vector<double>({2.5, 3.5})),
                 ralgo::vector<double>({2, 0}));

        CHECK_EQ(grid.cell_for_point(ralgo::vector<double>({15, 16})),
                 ralgo::vector<double>({3, 3}));

        CHECK_EQ(grid.cell_for_point(ralgo::vector<double>({2, 5})),
                 ralgo::vector<double>({1, 1}));
    }
}