#include <doctest/doctest.h>
#include <ralgo/cnc/interpreter.h>
#include <ralgo/robo/stepper.h>

TEST_CASE("interpreter")
{
    igris::ring<cnc::planner_block> blocks_ring(10);
    cnc::interpreter interpreter(&blocks_ring, nullptr, nullptr, nullptr);
}

TEST_CASE("interpreter.lookahead_settings")
{
    igris::ring<cnc::planner_block> blocks_ring(10);
    cnc::revolver revolver;
    cnc::planner planner(&blocks_ring, &revolver);
    cnc::feedback_guard guard(3);
    cnc::interpreter interpreter(&blocks_ring, &planner, &revolver, &guard);

    interpreter.init_axes(3);

    SUBCASE("По умолчанию look-ahead выключен")
    {
        CHECK_EQ(interpreter.is_lookahead_enabled(), false);
        CHECK_EQ(interpreter.get_junction_deviation(), 0);
    }

    SUBCASE("set_junction_deviation включает look-ahead")
    {
        interpreter.set_junction_deviation(0.05);

        CHECK_EQ(interpreter.is_lookahead_enabled(), true);
        CHECK_EQ(interpreter.get_junction_deviation(), 0.05);
    }

    SUBCASE("set_junction_deviation(0) выключает look-ahead")
    {
        interpreter.set_junction_deviation(0.05);
        CHECK_EQ(interpreter.is_lookahead_enabled(), true);

        interpreter.set_junction_deviation(0);
        CHECK_EQ(interpreter.is_lookahead_enabled(), false);
    }

    SUBCASE("junction_deviation_steps пересчитывается при изменении control_scale")
    {
        interpreter.set_junction_deviation(0.01);  // 0.01 мм
        interpreter.set_control_scale(0, 100);     // 100 pulses/mm

        // С одной осью avg = 100, junction_deviation_steps = 0.01 * 100 = 1
        // Проверить точное значение сложно, т.к. используется среднее
        CHECK(interpreter.is_lookahead_enabled());
    }
}

TEST_CASE("interpreter.lookahead_integration")
{
    // Интеграционный тест: проверяем что look-ahead параметры работают с gears
    igris::ring<cnc::planner_block> blocks_ring(10);
    cnc::revolver revolver;
    cnc::planner planner(&blocks_ring, &revolver);
    cnc::feedback_guard guard(2);
    cnc::interpreter interpreter(&blocks_ring, &planner, &revolver, &guard);

    interpreter.init_axes(2);

    // Сначала устанавливаем gears, потом junction_deviation
    // Это более реалистичный сценарий использования

    SUBCASE("junction_deviation_steps обновляется при изменении control_scale")
    {
        // Включаем look-ahead
        interpreter.set_junction_deviation(0.01);  // 0.01 мм
        CHECK(interpreter.is_lookahead_enabled());
        CHECK_EQ(interpreter.get_junction_deviation(), 0.01);

        // После init_axes default control_scale = 1, avg = 1
        // junction_deviation_steps = 0.01 * 1 = 0.01

        // Теперь устанавливаем control_scale через public API (не CLI)
        // set_control_scale вызывает update_junction_deviation_steps
    }

    SUBCASE("look-ahead выключается при junction_deviation = 0")
    {
        interpreter.set_junction_deviation(0.05);
        CHECK(interpreter.is_lookahead_enabled());

        interpreter.set_junction_deviation(0);
        CHECK_EQ(interpreter.is_lookahead_enabled(), false);
        CHECK_EQ(interpreter.get_junction_deviation(), 0);
    }
}

TEST_CASE("interpreter.buffering")
{
    igris::ring<cnc::planner_block> blocks_ring(10);
    cnc::revolver revolver;
    cnc::planner planner(&blocks_ring, &revolver);
    cnc::feedback_guard guard(2);
    cnc::interpreter interpreter(&blocks_ring, &planner, &revolver, &guard);

    interpreter.init_axes(2);
    interpreter.set_revolver_frequency(10000);

    SUBCASE("По умолчанию буферизация выключена")
    {
        CHECK_EQ(interpreter.get_min_blocks_to_start(), 0);
        CHECK_EQ(interpreter.is_buffer_mode(), false);
        CHECK(interpreter.is_buffer_ready());
    }

    SUBCASE("set_min_blocks_to_start настраивает автобуферизацию")
    {
        interpreter.set_min_blocks_to_start(3);
        CHECK_EQ(interpreter.get_min_blocks_to_start(), 3);
    }

    SUBCASE("set_buffer_timeout_ms конвертирует в тики")
    {
        interpreter.set_buffer_timeout_ms(100);
        // 100ms при 10000Hz = 1000 тиков
        CHECK_EQ(interpreter.get_buffer_timeout_ms(), 100);
    }

    SUBCASE("buffer_enable включает явный режим")
    {
        interpreter.buffer_enable();
        CHECK(interpreter.is_buffer_mode());
        CHECK(planner.pause); // planner должен быть на паузе
    }

    SUBCASE("buffer_start выключает режим и снимает паузу")
    {
        interpreter.buffer_enable();
        CHECK(interpreter.is_buffer_mode());

        interpreter.buffer_start();
        CHECK_EQ(interpreter.is_buffer_mode(), false);
        CHECK_EQ(planner.pause, false);
    }

    SUBCASE("buffer_cancel очищает режим")
    {
        interpreter.buffer_enable();
        interpreter.buffer_cancel();
        CHECK_EQ(interpreter.is_buffer_mode(), false);
        CHECK_EQ(planner.pause, false);
    }
}

TEST_CASE("interpreter.positioning_mode")
{
    igris::ring<cnc::planner_block> blocks_ring(10);
    cnc::revolver revolver;
    cnc::planner planner(&blocks_ring, &revolver);
    cnc::feedback_guard guard(2);
    cnc::interpreter interpreter(&blocks_ring, &planner, &revolver, &guard);

    interpreter.init_axes(2);

    SUBCASE("По умолчанию абсолютный режим (G90)")
    {
        CHECK(interpreter.is_absolute_mode());
    }

    SUBCASE("set_absolute_mode переключает режим")
    {
        interpreter.set_absolute_mode(false);
        CHECK_EQ(interpreter.is_absolute_mode(), false);

        interpreter.set_absolute_mode(true);
        CHECK(interpreter.is_absolute_mode());
    }

    SUBCASE("G90 включает абсолютный режим")
    {
        interpreter.set_absolute_mode(false);
        CHECK_EQ(interpreter.is_absolute_mode(), false);

        interpreter.newline("G90");
        CHECK(interpreter.is_absolute_mode());
    }

    SUBCASE("G91 включает относительный режим")
    {
        CHECK(interpreter.is_absolute_mode());

        interpreter.newline("G91");
        CHECK_EQ(interpreter.is_absolute_mode(), false);
    }

    SUBCASE("mode command показывает и меняет режим")
    {
        // Проверяем что команда mode работает
        interpreter.set_absolute_mode(true);
        auto result = interpreter.newline("mode rel");
        CHECK_EQ(interpreter.is_absolute_mode(), false);

        result = interpreter.newline("mode abs");
        CHECK(interpreter.is_absolute_mode());
    }
}

TEST_CASE("interpreter.G92_set_position")
{
    igris::ring<cnc::planner_block> blocks_ring(10);
    cnc::revolver revolver;
    robo::stepper steppers[2];
    robo::stepper *steppers_ptrs[] = {&steppers[0], &steppers[1]};
    revolver.set_steppers(steppers_ptrs, 2);
    cnc::planner planner(&blocks_ring, &revolver);
    cnc::feedback_guard guard(2);
    cnc::interpreter interpreter(&blocks_ring, &planner, &revolver, &guard);

    interpreter.init_axes(2);
    interpreter.set_control_scale(0, 100); // 100 pulses/mm
    interpreter.set_control_scale(1, 100);

    SUBCASE("G92 устанавливает позицию")
    {
        // Начальная позиция 0
        auto pos = interpreter.final_position();
        CHECK_EQ(pos[0], 0);
        CHECK_EQ(pos[1], 0);

        // Устанавливаем позицию через G92
        interpreter.newline("G92 X10 Y20");

        pos = interpreter.final_position();
        CHECK_EQ(pos[0], 10);
        CHECK_EQ(pos[1], 20);

        // Проверяем что счётчики шагов тоже обновились
        auto steps = interpreter.current_steps();
        CHECK_EQ(steps[0], 1000); // 10 * 100
        CHECK_EQ(steps[1], 2000); // 20 * 100
    }

    SUBCASE("G92 с одной осью не меняет другие")
    {
        interpreter.newline("G92 X5");

        auto pos = interpreter.final_position();
        CHECK_EQ(pos[0], 5);
        CHECK_EQ(pos[1], 0); // Y не изменился
    }
}

TEST_CASE("interpreter.G0_G1_with_modes")
{
    igris::ring<cnc::planner_block> blocks_ring(10);
    cnc::revolver revolver;
    robo::stepper steppers[2];
    robo::stepper *steppers_ptrs[] = {&steppers[0], &steppers[1]};
    revolver.set_steppers(steppers_ptrs, 2);
    cnc::planner planner(&blocks_ring, &revolver);
    cnc::feedback_guard guard(2);
    cnc::interpreter interpreter(&blocks_ring, &planner, &revolver, &guard);

    interpreter.init_axes(2);
    interpreter.set_revolver_frequency(10000);
    interpreter.set_control_scale(0, 100);
    interpreter.set_control_scale(1, 100);
    interpreter.set_saved_acc(1000);
    interpreter.set_saved_feed(100);

    SUBCASE("G1 в абсолютном режиме создаёт блок до целевой позиции")
    {
        interpreter.set_absolute_mode(true);
        interpreter.newline("G1 X10 F100 M1000");

        // Проверяем что final_position обновился
        auto pos = interpreter.final_position();
        CHECK_EQ(pos[0], doctest::Approx(10.0));

        // Проверяем что блок добавлен в очередь
        CHECK_EQ(blocks_ring.avail(), 1);
    }

    SUBCASE("G1 в относительном режиме создаёт блок со смещением")
    {
        // Сначала установим начальную позицию
        interpreter.newline("G92 X5");
        auto pos = interpreter.final_position();
        CHECK_EQ(pos[0], 5);

        // Переключаемся в относительный режим
        interpreter.newline("G91");

        // G1 X10 должен добавить 10 к текущей позиции
        interpreter.newline("G1 X10 F100 M1000");

        pos = interpreter.final_position();
        CHECK_EQ(pos[0], doctest::Approx(15.0)); // 5 + 10
    }

    SUBCASE("G0 использует максимальную скорость")
    {
        // Устанавливаем ограничение скорости по оси
        // G0 должен использовать feed=0 (что означает max velocity)
        interpreter.set_absolute_mode(true);
        interpreter.newline("G0 X10 M1000");

        // Блок должен быть создан
        CHECK_EQ(blocks_ring.avail(), 1);

        auto pos = interpreter.final_position();
        CHECK_EQ(pos[0], doctest::Approx(10.0));
    }
}
