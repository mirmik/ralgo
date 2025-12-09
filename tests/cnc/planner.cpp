#include <doctest/doctest.h>
#include <ralgo/cnc/planner.h>
#include <ralgo/robo/stepper.h>
#include <cmath>

TEST_CASE("planner.basic_setup")
{
    igris::ring<cnc::planner_block> blocks(10);
    cnc::revolver revolver;
    cnc::planner planner(&blocks, &revolver);

    planner.set_axes_count(3);
    CHECK_EQ(planner.total_axes(), 3);
    CHECK_EQ(planner.get_total_axes(), 3);
}

TEST_CASE("planner.pause_mode")
{
    igris::ring<cnc::planner_block> blocks(10);
    cnc::revolver revolver;
    cnc::planner planner(&blocks, &revolver);

    planner.set_axes_count(1);

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

TEST_CASE("planner.lookahead_basic")
{
    igris::ring<cnc::planner_block> blocks(10);
    cnc::revolver revolver;
    cnc::planner planner(&blocks, &revolver);

    planner.set_axes_count(2);

    cnc_float_type velocity = 1.0;       // steps/tick
    cnc_float_type acceleration = 0.1;   // steps/tick²
    cnc_float_type junction_deviation = 0.05;  // steps

    SUBCASE("Два блока по прямой - скорость сохраняется на стыке")
    {
        // Блок 1: движение по X
        cnc_float_type task1[] = {100, 0};
        ralgo::vector_view<cnc_float_type> view1{task1, 2};
        auto &block1 = blocks.head_place();
        block1.set_state(view1, 2, velocity, acceleration);
        blocks.move_head_one();

        // Блок 2: продолжение по X (то же направление)
        cnc_float_type task2[] = {100, 0};
        ralgo::vector_view<cnc_float_type> view2{task2, 2};
        auto &block2 = blocks.head_place();
        block2.set_state(view2, 2, velocity, acceleration);
        blocks.move_head_one();

        // Вызываем look-ahead
        planner.recalculate_block_velocities(junction_deviation);

        // При движении по прямой junction velocity должна быть равна nominal
        CHECK(block1.final_velocity == doctest::Approx(velocity).epsilon(0.01));
        CHECK(block2.start_velocity == doctest::Approx(velocity).epsilon(0.01));
    }

    SUBCASE("Два блока под углом 90° - скорость снижается на стыке")
    {
        // Блок 1: движение по X
        cnc_float_type task1[] = {100, 0};
        ralgo::vector_view<cnc_float_type> view1{task1, 2};
        auto &block1 = blocks.head_place();
        block1.set_state(view1, 2, velocity, acceleration);
        blocks.move_head_one();

        // Блок 2: движение по Y (поворот 90°)
        cnc_float_type task2[] = {0, 100};
        ralgo::vector_view<cnc_float_type> view2{task2, 2};
        auto &block2 = blocks.head_place();
        block2.set_state(view2, 2, velocity, acceleration);
        blocks.move_head_one();

        // Вызываем look-ahead
        planner.recalculate_block_velocities(junction_deviation);

        // При повороте на 90° скорость должна снизиться
        CHECK(block1.final_velocity < velocity);
        CHECK(block2.start_velocity < velocity);
        CHECK(block1.final_velocity == block2.start_velocity);
    }

    SUBCASE("Последний блок заканчивается с нулевой скоростью")
    {
        // Блок 1
        cnc_float_type task1[] = {100, 0};
        ralgo::vector_view<cnc_float_type> view1{task1, 2};
        auto &block1 = blocks.head_place();
        block1.set_state(view1, 2, velocity, acceleration);
        blocks.move_head_one();

        // Блок 2
        cnc_float_type task2[] = {100, 0};
        ralgo::vector_view<cnc_float_type> view2{task2, 2};
        auto &block2 = blocks.head_place();
        block2.set_state(view2, 2, velocity, acceleration);
        blocks.move_head_one();

        planner.recalculate_block_velocities(junction_deviation);

        // Последний блок должен закончиться остановкой
        CHECK_EQ(block2.final_velocity, 0);
    }

    SUBCASE("Первый блок начинается с нулевой скоростью")
    {
        cnc_float_type task1[] = {100, 0};
        ralgo::vector_view<cnc_float_type> view1{task1, 2};
        auto &block1 = blocks.head_place();
        block1.set_state(view1, 2, velocity, acceleration);
        blocks.move_head_one();

        planner.recalculate_block_velocities(junction_deviation);

        // Первый блок должен начаться с нуля (нет active_block)
        CHECK_EQ(block1.start_velocity, 0);
    }
}

TEST_CASE("planner.lookahead_backward_pass")
{
    igris::ring<cnc::planner_block> blocks(10);
    cnc::revolver revolver;
    cnc::planner planner(&blocks, &revolver);

    planner.set_axes_count(2);

    cnc_float_type velocity = 1.0;
    cnc_float_type acceleration = 0.1;
    cnc_float_type junction_deviation = 1.0;  // большое значение для теста

    SUBCASE("Backward pass снижает скорость если путь слишком короткий")
    {
        // Длинный блок, затем очень короткий
        // Короткий блок не сможет затормозить, если входит на высокой скорости

        // Блок 1: длинный
        cnc_float_type task1[] = {1000, 0};
        ralgo::vector_view<cnc_float_type> view1{task1, 2};
        auto &block1 = blocks.head_place();
        block1.set_state(view1, 2, velocity, acceleration);
        blocks.move_head_one();

        // Блок 2: очень короткий (не может затормозить от nominal до 0)
        cnc_float_type task2[] = {1, 0};  // всего 1 step
        ralgo::vector_view<cnc_float_type> view2{task2, 2};
        auto &block2 = blocks.head_place();
        block2.set_state(view2, 2, velocity, acceleration);
        blocks.move_head_one();

        planner.recalculate_block_velocities(junction_deviation);

        // Backward pass должен снизить junction velocity
        // чтобы блок 2 мог затормозить до 0
        // v² = 2*a*d => v = sqrt(2*0.1*1) = sqrt(0.2) ≈ 0.447
        cnc_float_type max_possible = sqrt(2 * acceleration * 1);
        CHECK(block2.start_velocity <= max_possible + 0.01);
    }
}
