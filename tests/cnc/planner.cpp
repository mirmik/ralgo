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

TEST_CASE("planner.block_lifecycle")
{
    igris::ring<cnc::planner_block> blocks(10);
    cnc::revolver revolver;
    cnc::planner planner(&blocks, &revolver);
    robo::stepper steppers[2];
    robo::stepper *steppers_ptrs[] = {&steppers[0], &steppers[1]};

    revolver.set_steppers(steppers_ptrs, 2);
    planner.set_axes_count(2);
    planner.set_gears({1000.0, 1000.0});

    CHECK(blocks.empty());
    CHECK_EQ(planner.active_block, nullptr);

    // Добавляем блок с большой длительностью, чтобы он не завершился за один serve()
    auto &block = blocks.head_place();
    block.blockno = 0;  // Important for planner validation
    block.nominal_velocity = 0.5;
    block.acceleration = 0.1;
    block.set_direction({1, 0});
    block.start_ic = 0;
    block.acceleration_before_ic = 500;
    block.deceleration_after_ic = 1000;
    block.block_finish_ic = 1500;
    block.fullpath = 10;
    block.exact_stop = 1;
    blocks.move_head_one();

    CHECK_FALSE(blocks.empty());
    CHECK_EQ(blocks.avail(), 1);

    // После serve блок должен стать активным
    planner.serve(false);
    CHECK(planner.active_block != nullptr);
}

TEST_CASE("planner.state_transitions")
{
    igris::ring<cnc::planner_block> blocks(10);
    cnc::revolver revolver;
    cnc::planner planner(&blocks, &revolver);
    robo::stepper steppers[1];
    robo::stepper *steppers_ptrs[] = {&steppers[0]};

    revolver.set_steppers(steppers_ptrs, 1);
    planner.set_axes_count(1);
    planner.set_gears({1000.0});

    // Создаём блок с чётким профилем: 2 такта разгон, 2 такта плоский, 2 такта
    // торможение
    auto &block = blocks.head_place();
    block.blockno = 0;
    block.nominal_velocity = 500.0;
    block.acceleration = 250.0;
    block.set_direction({1});
    block.start_ic = 0;
    block.acceleration_before_ic = 2;
    block.deceleration_after_ic = 4;
    block.block_finish_ic = 6;
    block.fullpath = 2;
    block.exact_stop = 1;
    blocks.move_head_one();

    // Начальное состояние
    CHECK_EQ(planner.state, 0);

    // Первая итерация — разгон
    planner.serve(false);
    CHECK(planner.active_block != nullptr);

    // Проверяем ускорения во время разгона
    CHECK_EQ(planner.accelerations[0], doctest::Approx(250.0));

    // Прогоняем до конца разгона
    planner.iteration();
    CHECK_EQ(planner.state, 0); // ещё разгон

    planner.iteration();
    CHECK_EQ(planner.state, 1); // переход на плоский участок
    CHECK_EQ(planner.accelerations[0], doctest::Approx(0.0));
}

TEST_CASE("planner.acceleration_evaluation")
{
    igris::ring<cnc::planner_block> blocks(10);
    cnc::revolver revolver;
    cnc::planner planner(&blocks, &revolver);
    robo::stepper steppers[2];
    robo::stepper *steppers_ptrs[] = {&steppers[0], &steppers[1]};

    revolver.set_steppers(steppers_ptrs, 2);
    planner.set_axes_count(2);
    planner.set_gears({1000.0, 1000.0});

    // Блок с направлением (0.6, 0.8) — нормализованный вектор
    auto &block = blocks.head_place();
    block.blockno = 0;
    block.nominal_velocity = 1000.0;
    block.acceleration = 500.0;
    block.set_direction({0.6, 0.8});
    block.start_ic = 0;
    block.acceleration_before_ic = 2;
    block.deceleration_after_ic = 4;
    block.block_finish_ic = 6;
    block.fullpath = 5000;
    block.exact_stop = 1;
    blocks.move_head_one();

    planner.serve(false);

    // Ускорения должны быть пропорциональны направлению
    CHECK_EQ(planner.accelerations[0], doctest::Approx(500.0 * 0.6));
    CHECK_EQ(planner.accelerations[1], doctest::Approx(500.0 * 0.8));
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

TEST_CASE("planner.clear")
{
    igris::ring<cnc::planner_block> blocks(10);
    cnc::revolver revolver;
    cnc::planner planner(&blocks, &revolver);
    robo::stepper steppers[1];
    robo::stepper *steppers_ptrs[] = {&steppers[0]};

    revolver.set_steppers(steppers_ptrs, 1);
    planner.set_axes_count(1);
    planner.set_gears({1000.0});

    // Добавляем блок и запускаем
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

    planner.serve(false);
    CHECK(planner.active_block != nullptr);

    // Очищаем
    planner.clear();

    CHECK(planner.active_block == nullptr);
    CHECK(blocks.empty());
    CHECK_EQ(planner.accelerations[0], 0);
    CHECK_EQ(planner.state, 0);
}
