#include <chrono>
#include <doctest/doctest.h>
#include <mutex>
#include <ralgo/cnc/interpreter.h>
#include <ralgo/cnc/planner.h>
#include <ralgo/cnc/revolver.h>
#include <ralgo/global_protection.h>
#include <thread>

igris::ring<cnc::planner_block> blocks{40};
igris::ring<cnc::control_shift> shifts{400};
cnc::planner planner(&blocks, &shifts);
cnc::revolver revolver(&shifts, &blocks, &planner);
cnc::interpreter interpreter(&blocks, &planner, &revolver);
robo::stepper steppers[3];
robo::stepper *steppers_ptrs[] = {&steppers[0], &steppers[1], &steppers[2]};
std::mutex mtx;
volatile bool cancel_token = false;

using std::chrono::operator""ms;
using std::chrono::operator""us;

auto now()
{
    return std::chrono::steady_clock::now();
}

void planner_thread_function()
{
    auto awake = now();
    while (1)
    {
        if (cancel_token)
            return;
        awake += 1ms;
        std::this_thread::sleep_until(awake);
        {
            std::lock_guard lock(mtx);
            planner.serve();
        }
    }
}

void revolver_thread_function()
{
    auto awake = now();
    while (1)
    {
        if (cancel_token)
            return;
        awake += 200us;
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
    interpreter.init_axes(3);
    interpreter.set_scale(ralgo::vector<double>{1, 1, 1});
    planner.set_gears({1000, 1000, 1000});
    interpreter.set_revolver_frequency(5000);
    revolver.set_steppers(steppers_ptrs, 3);
    ralgo::global_protection = false;

    planner_thread = std::thread(planner_thread_function);
    revolver_thread = std::thread(revolver_thread_function);

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
}