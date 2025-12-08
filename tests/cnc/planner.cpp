#include <doctest/doctest.h>
#include <ralgo/cnc/planner.h>
#include <ralgo/robo/stepper.h>

TEST_CASE("planner.basic_setup")
{
    igris::ring<cnc::planner_block> blocks(10);
    cnc::revolver revolver;
    cnc::planner planner(&blocks, &revolver);

    planner.set_axes_count(3);
    CHECK_EQ(planner.total_axes(), 3);

    planner.set_gears({100, 200, 300});
    auto gears = planner.get_gears();
    CHECK_EQ(gears[0], 100);
    CHECK_EQ(gears[1], 200);
    CHECK_EQ(gears[2], 300);
}

TEST_CASE("planner.pause_mode")
{
    igris::ring<cnc::planner_block> blocks(10);
    cnc::revolver revolver;
    cnc::planner planner(&blocks, &revolver);

    planner.set_axes_count(1);
    planner.set_gears({1000.0});

    auto &block = blocks.head_place();
    block.blockno = 0;
    block.nominal_velocity = 500.0;
    block.acceleration = 100.0;
    block.set_direction({1});
    block.start_ic = 0;
    block.acceleration_before_ic = 5;
    block.deceleration_after_ic = 10;
    block.block_finish_ic = 15;
    block.exact_stop = 1;
    blocks.move_head_one();

    int64_t ic_before = planner.iteration_counter;

    // Включаем паузу
    planner.set_pause_mode(true);
    planner.serve(false);

    // iteration_counter не должен измениться
    CHECK_EQ(planner.iteration_counter, ic_before);

    // Выключаем паузу
    planner.set_pause_mode(false);
    planner.serve(false);

    // Теперь должен продвинуться
    CHECK_GT(planner.iteration_counter, ic_before);
}
