#include <doctest/doctest.h>
#include <ralgo/cartesian_grid.h>

TEST_CASE("cartesian_grid")
{
    std::vector<std::vector<double>> grid({{1, 2, 3}, {4, 5}});

    {
        CHECK_EQ(ralgo::number_of_cartesian_grid_interval_for_point(grid,
                                                                    {0.5, 0.5}),
                 (std::vector<size_t>{0, 0}));

        CHECK_EQ(ralgo::number_of_cartesian_grid_interval_for_point(grid,
                                                                    {1.5, 0.5}),
                 (std::vector<size_t>{1, 0}));

        CHECK_EQ(ralgo::number_of_cartesian_grid_interval_for_point(grid,
                                                                    {2.5, 0.5}),

                 (std::vector<size_t>{2, 0}));

        CHECK_EQ(ralgo::number_of_cartesian_grid_interval_for_point(grid,
                                                                    {3.5, 0.5}),

                 (std::vector<size_t>{3, 0}));

        CHECK_EQ(ralgo::number_of_cartesian_grid_interval_for_point(grid,
                                                                    {0.5, 4.5}),

                 (std::vector<size_t>{0, 1}));

        CHECK_EQ(ralgo::number_of_cartesian_grid_interval_for_point(grid,
                                                                    {1.5, 4.5}),

                 (std::vector<size_t>{1, 1}));

        CHECK_EQ(ralgo::number_of_cartesian_grid_interval_for_point(grid,
                                                                    {2.5, 4.5}),

                 (std::vector<size_t>{2, 1}));

        CHECK_EQ(ralgo::number_of_cartesian_grid_interval_for_point(grid,
                                                                    {3.5, 4.5}),

                 (std::vector<size_t>{3, 1}));

        CHECK_EQ(ralgo::number_of_cartesian_grid_interval_for_point(grid,
                                                                    {0.5, 5.5}),

                 (std::vector<size_t>{0, 2}));
    }
}