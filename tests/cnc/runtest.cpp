
#include <chrono>
#include <doctest/doctest.h>
#include <mutex>
#include <thread>

#include <ralgo/cnc/interpreter.h>
#include <ralgo/cnc/planner.h>
#include <ralgo/cnc/revolver.h>
#include <ralgo/freq_invoke_imitation.h>
#include <ralgo/global_protection.h>

nos::log::logger logger;
using namespace std::chrono_literals;

TEST_CASE("cnc.runtest with control_scale assigned one")
{
    ralgo::set_logger(&logger);
    igris::ring<cnc::planner_block> blocks{40};
    cnc::revolver revolver;
    cnc::planner planner(&blocks, &revolver);
    cnc::interpreter interpreter(&blocks, &planner, &revolver, nullptr);
    robo::stepper steppers[3];
    robo::stepper *steppers_ptrs[] = {&steppers[0], &steppers[1], &steppers[2]};
    cnc_float_type freq = 10000.0;

    ralgo::freq_invoke_imitation imitator = {
        {1ms, [&]() { planner.serve(); }},
        {100us, [&]() { revolver.serve(); }},
    };

    interpreter.init_axes(3);
    // Set control_scale = 1 for all axes (1 pulse per unit)
    for (int i = 0; i < 3; i++)
        interpreter.set_control_scale(i, 1);
    interpreter.set_revolver_frequency(freq);
    revolver.set_steppers(steppers_ptrs, 3);
    ralgo::global_protection = false;

    auto responce = interpreter.newline("cnc G1 X10000 F10000 M10000\n");
    CHECK_EQ(responce, "");

    auto last_block = interpreter.last_block();

    CHECK_EQ(last_block.axdist[0], 10000);
    CHECK_EQ(last_block.axdist[1], 0);
    CHECK_EQ(last_block.axdist[2], 0);
    CHECK_EQ(last_block.direction()[0], 1);
    CHECK_EQ(last_block.direction()[1], 0);
    CHECK_EQ(last_block.direction()[2], 0);
    CHECK_EQ(last_block.nominal_velocity, 10000.0 / freq);
    CHECK_EQ(last_block.acceleration, 10000.0f / freq / freq);
    CHECK_EQ(last_block.fullpath, 10000);

    std::vector<
        std::tuple<std::chrono::milliseconds, cnc_float_type, cnc_float_type>>
        points{
            {100ms, 0, 20},       {200ms, 40, 60},      {300ms, 190, 200},
            {400ms, 440, 450},    {500ms, 790, 800},    {600ms, 1240, 1250},
            {700ms, 1780, 1800},  {800ms, 2440, 2460},  {900ms, 3180, 3200},
            {1000ms, 4030, 4050}, {1100ms, 4980, 5000}, {1200ms, 5930, 5950},
            {1300ms, 6780, 6820}, {1400ms, 7530, 7550}, {1500ms, 8180, 8200},
            {1600ms, 8730, 8750}, {1700ms, 9180, 9200}, {1800ms, 9530, 9550},
            {1900ms, 9780, 9800}, {2000ms, 9940, 10000}, {2100ms, 9990, 10000}};

    auto lasttime = 0ns;
    for (auto &p : points)
    {
        auto steps = steppers[0].steps();
        CHECK_LE(steps, std::get<2>(p));
        CHECK_GE(steps, std::get<1>(p));
        imitator.timeskip(std::get<0>(p) - lasttime);
        lasttime = std::get<0>(p);
    }

    CHECK_FALSE(planner.is_dda_overflow_detected());
    // Allow 0.1% tolerance for DDA rounding
    CHECK_GE(interpreter.final_position()[0], 9990);
    CHECK_LE(interpreter.final_position()[0], 10010);
}
