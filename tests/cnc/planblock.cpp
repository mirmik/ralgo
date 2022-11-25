#include <doctest/doctest.h>
#include <ralgo/cnc/planner.h>

TEST_CASE("planblock")
{
    cnc::planner_block block;
    double task[] = {10, 20};
    double mults[] = {1, 1};
    ralgo::vector_view<double> task_view{task, 2};
    ralgo::vector_view<double> mults_view{mults, 2};
    block.set_state(task_view, 2, 0.5, 0.25, mults_view);
    CHECK_EQ(block.is_triangle(), false);
}

TEST_CASE("planblock.triangle")
{
    cnc::planner_block block;
    double task[] = {10, 0};
    double mults[] = {1, 1};
    ralgo::vector_view<double> task_view{task, 2};
    ralgo::vector_view<double> mults_view{mults, 2};
    block.set_state(task_view, 2, 0.5, 0.01, mults_view);
    CHECK_EQ(block.is_triangle(), true);
}
