#include <doctest/doctest.h>
#include <ralgo/cnc/planner.h>

TEST_CASE("planblock")
{
    cnc::planner_block block;
    double task[] = {10, 20};
    double velocity = 0.5;
    double acceleration = 0.25;
    ralgo::vector_view<double> task_view{task, 2};
    block.set_state(task_view, 2, velocity, acceleration);
    CHECK_EQ(block.is_triangle(), false);
    CHECK_EQ(block.block_finish_ic, 47);
}

TEST_CASE("planblock.triangle")
{
    cnc::planner_block block;
    double task[] = {10, 0};
    double velocity = 0.5;
    double acceleration = 0.01;
    ralgo::vector_view<double> task_view{task, 2};
    block.set_state(task_view, 2, velocity, acceleration);
    CHECK_EQ(block.is_triangle(), true);
    CHECK_EQ(block.block_finish_ic, 64);
}

TEST_CASE("planblock.stop_pattern")
{
    cnc::planner_block block;
    int total_axes = 2;
    double velocity = 0.5;
    double acceleration = 0.25;
    ralgo::vector<double> direction({1, 0});

    block.set_stop_pattern(total_axes, velocity, acceleration, direction);
    CHECK_EQ(block.nominal_velocity, 0.5);
    CHECK_EQ(block.acceleration, 0.25);
    CHECK_EQ(block.block_finish_ic, 2);
    CHECK_EQ(block.fullpath, 0.5);
}
