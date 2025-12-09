#include <doctest/doctest.h>
#include <ralgo/cnc/planner.h>

TEST_CASE("planblock")
{
    cnc::planner_block block;
    cnc_float_type task[] = {10, 20};
    cnc_float_type velocity = 0.5;
    cnc_float_type acceleration = 0.25;
    ralgo::vector_view<cnc_float_type> task_view{task, 2};
    block.set_state(task_view, 2, velocity, acceleration);
    CHECK_EQ(block.is_triangle(), false);
    CHECK_EQ(block.block_finish_ic, 47);
}

TEST_CASE("planblock.triangle")
{
    cnc::planner_block block;
    cnc_float_type task[] = {10, 0};
    cnc_float_type velocity = 0.5;
    cnc_float_type acceleration = 0.01;
    ralgo::vector_view<cnc_float_type> task_view{task, 2};
    block.set_state(task_view, 2, velocity, acceleration);
    CHECK_EQ(block.is_triangle(), true);
    CHECK_EQ(block.block_finish_ic, 64);
}

TEST_CASE("planblock.stop_pattern")
{
    cnc::planner_block block;
    int total_axes = 2;
    cnc_float_type velocity = 0.5;
    cnc_float_type acceleration = 0.25;
    ralgo::vector<cnc_float_type> direction({1, 0});

    block.set_stop_pattern(total_axes, velocity, acceleration, direction);
    CHECK_EQ(block.nominal_velocity, 0.5);
    CHECK_EQ(block.acceleration, 0.25);
    CHECK_EQ(block.block_finish_ic, 2);
    CHECK_EQ(block.fullpath, 0.5);
}

TEST_CASE("planblock.set_state_with_junction_velocities")
{
    cnc::planner_block block;
    cnc_float_type task[] = {100, 0};  // 100 steps по X
    cnc_float_type velocity = 1.0;      // steps/tick
    cnc_float_type acceleration = 0.1;  // steps/tick²
    ralgo::vector_view<cnc_float_type> task_view{task, 2};

    SUBCASE("С нулевыми граничными скоростями (старое поведение)")
    {
        block.set_state(task_view, 2, velocity, acceleration, 0, 0);

        CHECK_EQ(block.start_velocity, 0);
        CHECK_EQ(block.final_velocity, 0);
        CHECK(block.nominal_velocity > 0);
        CHECK(block.block_finish_ic > 0);
    }

    SUBCASE("С ненулевой start_velocity")
    {
        block.set_state(task_view, 2, velocity, acceleration, 0.5, 0);

        CHECK_EQ(block.start_velocity, 0.5);
        CHECK_EQ(block.final_velocity, 0);
        // Время разгона должно быть меньше, т.к. начинаем с 0.5
        CHECK(block.acceleration_before_ic < 10);  // меньше чем при start=0
    }

    SUBCASE("С ненулевой final_velocity")
    {
        block.set_state(task_view, 2, velocity, acceleration, 0, 0.5);

        CHECK_EQ(block.start_velocity, 0);
        CHECK_EQ(block.final_velocity, 0.5);
    }

    SUBCASE("С обеими ненулевыми скоростями")
    {
        block.set_state(task_view, 2, velocity, acceleration, 0.3, 0.4);

        CHECK_EQ(block.start_velocity, 0.3);
        CHECK_EQ(block.final_velocity, 0.4);
        CHECK(block.nominal_velocity >= 0.4);  // nominal >= max(start, final)
    }

    SUBCASE("Граничные скорости ограничены nominal")
    {
        // Пробуем установить start > nominal
        block.set_state(task_view, 2, velocity, acceleration, 2.0, 1.5);

        // Должны быть ограничены до velocity
        CHECK_EQ(block.start_velocity, velocity);
        CHECK_EQ(block.final_velocity, velocity);
    }
}

TEST_CASE("planblock.recalculate_timing")
{
    cnc::planner_block block;
    cnc_float_type task[] = {100, 0};
    cnc_float_type velocity = 1.0;
    cnc_float_type acceleration = 0.1;
    ralgo::vector_view<cnc_float_type> task_view{task, 2};

    SUBCASE("Пересчёт после изменения скоростей")
    {
        // Сначала создаём блок с нулевыми скоростями
        block.set_state(task_view, 2, velocity, acceleration, 0, 0);
        int64_t original_finish = block.block_finish_ic;

        // Меняем граничные скорости
        block.set_junction_velocities(0.5, 0.5);

        // Время должно уменьшиться (меньше разгона и торможения)
        CHECK(block.block_finish_ic < original_finish);
        CHECK_EQ(block.start_velocity, 0.5);
        CHECK_EQ(block.final_velocity, 0.5);
    }

    SUBCASE("Треугольный профиль при коротком пути")
    {
        // Короткий путь с нулевыми граничными скоростями
        // должен дать треугольный профиль (не успеваем разогнаться до nominal)
        cnc_float_type short_task[] = {1, 0};  // очень короткий путь
        ralgo::vector_view<cnc_float_type> short_view{short_task, 2};

        block.set_state(short_view, 2, velocity, acceleration, 0, 0);

        // При коротком пути должен получиться треугольный профиль
        CHECK(block.is_triangle());
    }

    SUBCASE("При равных start и final скоростях - чистый круиз")
    {
        // Если start == final == nominal, то это чистый круиз без разгона/торможения
        block.set_state(task_view, 2, velocity, acceleration, velocity, velocity);

        CHECK_EQ(block.start_velocity, velocity);
        CHECK_EQ(block.final_velocity, velocity);
        // Время разгона и торможения = 0
        CHECK_EQ(block.acceleration_before_ic, 0);
    }
}

TEST_CASE("planblock.junction_velocity")
{
    cnc_float_type nominal_velocity = 1.0;
    cnc_float_type acceleration = 0.1;
    cnc_float_type junction_deviation = 0.05;
    int axes = 2;

    SUBCASE("Прямая линия - скорость сохраняется")
    {
        std::array<cnc_float_type, NMAX_AXES> dir1 = {1.0, 0.0};
        std::array<cnc_float_type, NMAX_AXES> dir2 = {1.0, 0.0};

        cnc_float_type v = cnc::planner_block::calculate_junction_velocity(
            dir1, dir2, nominal_velocity, acceleration, junction_deviation,
            axes);

        CHECK_EQ(v, nominal_velocity);
    }

    SUBCASE("Угол 90° - скорость снижается")
    {
        std::array<cnc_float_type, NMAX_AXES> dir1 = {1.0, 0.0};
        std::array<cnc_float_type, NMAX_AXES> dir2 = {0.0, 1.0};

        cnc_float_type v = cnc::planner_block::calculate_junction_velocity(
            dir1, dir2, nominal_velocity, acceleration, junction_deviation,
            axes);

        // cos(90°) = 0, sin(45°) = 0.707
        // v = sqrt(0.1 * 0.05 / 0.707) ≈ 0.084
        CHECK(v < nominal_velocity);
        CHECK(v > 0);
        CHECK(v == doctest::Approx(0.0841).epsilon(0.01));
    }

    SUBCASE("Угол 180° (разворот) - скорость = 0")
    {
        std::array<cnc_float_type, NMAX_AXES> dir1 = {1.0, 0.0};
        std::array<cnc_float_type, NMAX_AXES> dir2 = {-1.0, 0.0};

        cnc_float_type v = cnc::planner_block::calculate_junction_velocity(
            dir1, dir2, nominal_velocity, acceleration, junction_deviation,
            axes);

        CHECK_EQ(v, 0);
    }

    SUBCASE("Угол 45° - скорость между nominal и 90°")
    {
        // dir2 = (cos(45°), sin(45°)) = (0.707, 0.707)
        cnc_float_type sqrt2_2 = 0.7071067811865476;
        std::array<cnc_float_type, NMAX_AXES> dir1 = {1.0, 0.0};
        std::array<cnc_float_type, NMAX_AXES> dir2 = {sqrt2_2, sqrt2_2};

        cnc_float_type v = cnc::planner_block::calculate_junction_velocity(
            dir1, dir2, nominal_velocity, acceleration, junction_deviation,
            axes);

        // cos(45°) = 0.707, угол меньше => скорость выше чем при 90°
        CHECK(v < nominal_velocity);
        CHECK(v > 0.0841);  // выше чем при 90°
    }

    SUBCASE("Результат ограничен nominal_velocity")
    {
        // При очень большом junction_deviation результат не должен превышать
        // nominal
        std::array<cnc_float_type, NMAX_AXES> dir1 = {1.0, 0.0};
        std::array<cnc_float_type, NMAX_AXES> dir2 = {0.0, 1.0};

        cnc_float_type v = cnc::planner_block::calculate_junction_velocity(
            dir1, dir2, nominal_velocity, acceleration,
            1000.0,  // очень большое отклонение
            axes);

        CHECK_EQ(v, nominal_velocity);
    }
}
