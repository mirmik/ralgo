#include <chrono>
#include <doctest/doctest.h>
#include <mutex>
#include <ralgo/cnc/interpreter.h>
#include <ralgo/cnc/planner.h>
#include <ralgo/cnc/revolver.h>
#include <ralgo/global_protection.h>
#include <thread>

nos::log::logger logger;

std::mutex mtx;
using std::chrono::operator""ms;
using std::chrono::operator""us;

auto now()
{
    return std::chrono::steady_clock::now();
}

void planner_thread_function(cnc::planner &planner,
                             volatile bool &cancel_token,
                             std::chrono::nanoseconds us)
{
    auto awake = now();
    while (1)
    {
        if (cancel_token)
            return;
        awake += us;
        std::this_thread::sleep_until(awake);
        {
            std::lock_guard lock(mtx);
            planner.serve();
        }
    }
}

void revolver_thread_function(cnc::revolver &revolver,
                              volatile bool &cancel_token,
                              std::chrono::nanoseconds us)
{
    auto awake = now();
    while (1)
    {
        if (cancel_token)
            return;
        awake += us;
        std::this_thread::sleep_until(awake);
        {
            std::lock_guard lock(mtx);
            revolver.serve();
        }
    }
}

std::thread revolver_thread;
std::thread planner_thread;

TEST_CASE("cnc.runtest")
{
    ralgo::set_logger(&logger);
    igris::ring<cnc::planner_block> blocks{40};
    igris::ring<cnc::control_shift> shifts{400};
    cnc::planner planner(&blocks, &shifts);
    cnc::revolver revolver(&shifts, &blocks, &planner);
    cnc::interpreter interpreter(&blocks, &planner, &revolver);
    robo::stepper steppers[3];
    robo::stepper *steppers_ptrs[] = {&steppers[0], &steppers[1], &steppers[2]};
    volatile bool cancel_token = false;

    interpreter.init_axes(3);
    interpreter.set_scale(ralgo::vector<double>{1, 1, 1});
    planner.set_gears({1000, 1000, 1000});
    interpreter.set_revolver_frequency(5000);
    revolver.set_steppers(steppers_ptrs, 3);
    ralgo::global_protection = false;

    planner_thread = std::thread(planner_thread_function,
                                 std::ref(planner),
                                 std::ref(cancel_token),
                                 1ms);
    revolver_thread = std::thread(revolver_thread_function,
                                  std::ref(revolver),
                                  std::ref(cancel_token),
                                  200us);

    auto responce = interpreter.newline("cnc G1 X10000 F10000 M10000\n");
    CHECK_EQ(responce, "");

    auto last_block = interpreter.last_block();

    CHECK_EQ(last_block.axdist[0], 10000);
    CHECK_EQ(last_block.axdist[1], 0);
    CHECK_EQ(last_block.axdist[2], 0);
    CHECK_EQ(last_block.multipliers[0], 1);
    CHECK_EQ(last_block.multipliers[1], 0);
    CHECK_EQ(last_block.multipliers[2], 0);
    CHECK_EQ(last_block.nominal_velocity, 10000.0 / 5000.0);
    CHECK_EQ(last_block.acceleration, 10000.0 / 5000.0 / 5000.0);
    CHECK_EQ(last_block.fullpath, 10000);

    auto start_time = now();

    CHECK_EQ(steppers[0].steps(), 0);
    std::this_thread::sleep_until(start_time + 500ms);

    CHECK_EQ(steppers[0].steps(), 1);
    std::this_thread::sleep_until(start_time + 600ms);

    CHECK_EQ(steppers[0].steps(), 1);
    std::this_thread::sleep_until(start_time + 700ms);

    CHECK_EQ(steppers[0].steps(), 2);
    std::this_thread::sleep_until(start_time + 800ms);

    CHECK_EQ(steppers[0].steps(), 3);
    std::this_thread::sleep_until(start_time + 900ms);

    CHECK_EQ(steppers[0].steps(), 4);
    std::this_thread::sleep_until(start_time + 1000ms);

    CHECK_EQ(steppers[0].steps(), 5);
    std::this_thread::sleep_until(start_time + 1100ms);

    CHECK_EQ(steppers[0].steps(), 6);
    std::this_thread::sleep_until(start_time + 1200ms);

    CHECK_EQ(steppers[0].steps(), 6);
    std::this_thread::sleep_until(start_time + 1300ms);

    CHECK_EQ(steppers[0].steps(), 7);
    std::this_thread::sleep_until(start_time + 1400ms);

    CHECK_EQ(steppers[0].steps(), 8);
    std::this_thread::sleep_until(start_time + 1500ms);

    CHECK_EQ(steppers[0].steps(), 8);
    std::this_thread::sleep_until(start_time + 1600ms);

    CHECK_EQ(steppers[0].steps(), 9);
    std::this_thread::sleep_until(start_time + 1700ms);

    CHECK_EQ(steppers[0].steps(), 9);
    std::this_thread::sleep_until(start_time + 1800ms);

    CHECK_EQ(steppers[0].steps(), 9);
    std::this_thread::sleep_until(start_time + 1900ms);

    CHECK_EQ(steppers[0].steps(), 10);
    std::this_thread::sleep_until(start_time + 2000ms);

    cancel_token = true;
    revolver_thread.join();
    planner_thread.join();
    CHECK_FALSE(planner.is_dda_overflow_detected());
}

TEST_CASE("cnc.runtest with gears assigned one")
{
    ralgo::set_logger(&logger);
    igris::ring<cnc::planner_block> blocks{40};
    igris::ring<cnc::control_shift> shifts{400};
    cnc::planner planner(&blocks, &shifts);
    cnc::revolver revolver(&shifts, &blocks, &planner);
    cnc::interpreter interpreter(&blocks, &planner, &revolver);
    robo::stepper steppers[3];
    robo::stepper *steppers_ptrs[] = {&steppers[0], &steppers[1], &steppers[2]};
    volatile bool cancel_token = false;
    double freq = 10000;

    interpreter.init_axes(3);
    interpreter.set_scale(ralgo::vector<double>{1, 1, 1});
    planner.set_gears({1, 1, 1});
    interpreter.set_revolver_frequency(freq);
    revolver.set_steppers(steppers_ptrs, 3);
    ralgo::global_protection = false;

    planner_thread = std::thread(planner_thread_function,
                                 std::ref(planner),
                                 std::ref(cancel_token),
                                 1ms);
    revolver_thread = std::thread(
        revolver_thread_function,
        std::ref(revolver),
        std::ref(cancel_token),
        std::chrono::microseconds(static_cast<int>(1000000 / freq)));

    auto responce = interpreter.newline("cnc G1 X10000 F10000 M10000\n");
    CHECK_EQ(responce, "");

    auto last_block = interpreter.last_block();

    CHECK_EQ(last_block.axdist[0], 10000);
    CHECK_EQ(last_block.axdist[1], 0);
    CHECK_EQ(last_block.axdist[2], 0);
    CHECK_EQ(last_block.multipliers[0], 1);
    CHECK_EQ(last_block.multipliers[1], 0);
    CHECK_EQ(last_block.multipliers[2], 0);
    CHECK_EQ(last_block.nominal_velocity, 10000.0 / freq);
    CHECK_EQ(last_block.acceleration, 10000.0 / freq / freq);
    CHECK_EQ(last_block.fullpath, 10000);

    auto start_time = now();

    std::vector<std::tuple<std::chrono::milliseconds, double, double>> points{
        {100ms, 0, 20},       {200ms, 40, 60},       {300ms, 190, 200},
        {400ms, 440, 450},    {500ms, 790, 800},     {600ms, 1240, 1250},
        {700ms, 1780, 1800},  {800ms, 2440, 2450},   {900ms, 3180, 3200},
        {1000ms, 4030, 4050}, {1100ms, 4980, 5000},  {1200ms, 5930, 5950},
        {1300ms, 6780, 6800}, {1400ms, 7530, 7550},  {1500ms, 8180, 8200},
        {1600ms, 8730, 8750}, {1700ms, 9180, 9200},  {1800ms, 9530, 9550},
        {1900ms, 9780, 9800}, {2000ms, 9940, 10000}, {2100ms, 10000, 10000}};

    for (auto &p : points)
    {
        CHECK_LE(steppers[0].steps(), std::get<2>(p));
        CHECK_GE(steppers[0].steps(), std::get<1>(p));
        std::this_thread::sleep_until(start_time + std::get<0>(p));
    }

    cancel_token = true;
    revolver_thread.join();
    planner_thread.join();
    CHECK_FALSE(planner.is_dda_overflow_detected());
}

TEST_CASE("cnc.runtest with gears assigned ten")
{
    ralgo::set_logger(&logger);
    igris::ring<cnc::planner_block> blocks{40};
    igris::ring<cnc::control_shift> shifts{400};
    cnc::planner planner(&blocks, &shifts);
    cnc::revolver revolver(&shifts, &blocks, &planner);
    cnc::interpreter interpreter(&blocks, &planner, &revolver);
    robo::stepper steppers[3];
    robo::stepper *steppers_ptrs[] = {&steppers[0], &steppers[1], &steppers[2]};
    volatile bool cancel_token = false;

    interpreter.init_axes(3);
    interpreter.set_scale(ralgo::vector<double>{1, 1, 1});
    planner.set_gears({10, 10, 10});
    interpreter.set_revolver_frequency(5000);
    revolver.set_steppers(steppers_ptrs, 3);
    ralgo::global_protection = false;

    planner_thread = std::thread(planner_thread_function,
                                 std::ref(planner),
                                 std::ref(cancel_token),
                                 1ms);
    revolver_thread = std::thread(revolver_thread_function,
                                  std::ref(revolver),
                                  std::ref(cancel_token),
                                  200us);

    auto responce = interpreter.newline("cnc G1 X10000 F10000 M10000\n");
    CHECK_EQ(responce, "");

    auto last_block = interpreter.last_block();

    CHECK_EQ(last_block.axdist[0], 10000);
    CHECK_EQ(last_block.axdist[1], 0);
    CHECK_EQ(last_block.axdist[2], 0);
    CHECK_EQ(last_block.multipliers[0], 1);
    CHECK_EQ(last_block.multipliers[1], 0);
    CHECK_EQ(last_block.multipliers[2], 0);
    CHECK_EQ(last_block.nominal_velocity, 10000.0 / 5000.0);
    CHECK_EQ(last_block.acceleration, 10000.0 / 5000.0 / 5000.0);
    CHECK_EQ(last_block.fullpath, 10000);

    auto start_time = now();

    std::vector<std::tuple<std::chrono::milliseconds, double, double>> points{
        {100ms, 0, 2},      {200ms, 4, 6},       {300ms, 19, 20},
        {400ms, 44, 45},    {500ms, 79, 80},     {600ms, 124, 125},
        {700ms, 178, 180},  {800ms, 244, 245},   {900ms, 318, 320},
        {1000ms, 403, 405}, {1100ms, 498, 500},  {1200ms, 593, 595},
        {1300ms, 678, 680}, {1400ms, 753, 755},  {1500ms, 818, 820},
        {1600ms, 873, 875}, {1700ms, 918, 920},  {1800ms, 953, 955},
        {1900ms, 978, 980}, {2000ms, 994, 1000}, {2100ms, 1000, 1000}};

    for (auto &p : points)
    {
        CHECK_LE(steppers[0].steps(), std::get<2>(p));
        CHECK_GE(steppers[0].steps(), std::get<1>(p));
        std::this_thread::sleep_until(start_time + std::get<0>(p));
    }

    cancel_token = true;
    revolver_thread.join();
    planner_thread.join();
    CHECK_FALSE(planner.is_dda_overflow_detected());
}