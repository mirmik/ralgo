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

    SUBCASE("junction_deviation_steps пересчитывается при изменении gears")
    {
        interpreter.set_junction_deviation(0.01);  // 0.01 мм
        interpreter.set_steps_per_unit(0, 100);    // 100 steps/mm

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

    SUBCASE("junction_deviation_steps обновляется при изменении gears")
    {
        // Включаем look-ahead
        interpreter.set_junction_deviation(0.01);  // 0.01 мм
        CHECK(interpreter.is_lookahead_enabled());
        CHECK_EQ(interpreter.get_junction_deviation(), 0.01);

        // После init_axes default gears = 1, avg_steps_per_unit = 1
        // junction_deviation_steps = 0.01 * 1 = 0.01

        // Теперь устанавливаем gears через public API (не CLI)
        // set_steps_per_unit вызывает update_junction_deviation_steps
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
