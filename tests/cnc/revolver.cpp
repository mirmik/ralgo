#include <doctest/doctest.h>
#include <ralgo/cnc/revolver.h>
#include <ralgo/robo/stepper.h>

// New DDA works purely in steps/tick units.
// Acceleration is in steps/tick², velocity in steps/tick.
// FIXED_POINT_MUL = 2^24 provides sub-step precision.

TEST_CASE("revolver.basic_setup")
{
    cnc::revolver revolver;
    robo::stepper steppers[3];
    robo::stepper *steppers_ptrs[] = {&steppers[0], &steppers[1], &steppers[2]};

    revolver.set_steppers(steppers_ptrs, 3);

    CHECK_EQ(revolver.steppers_total, 3);
    CHECK(revolver.is_empty());
}

TEST_CASE("revolver.task_queue")
{
    cnc::revolver revolver;
    robo::stepper steppers[2];
    robo::stepper *steppers_ptrs[] = {&steppers[0], &steppers[1]};

    revolver.set_steppers(steppers_ptrs, 2);

    CHECK(revolver.is_empty());

    // Add task with acceleration in steps/tick²
    std::array<cnc_float_type, NMAX_AXES> accels = {};
    accels[0] = 0.1;  // 0.1 steps/tick²
    accels[1] = 0.2;
    revolver.task_queue.emplace(accels, 10);

    CHECK_FALSE(revolver.is_empty());
    CHECK_EQ(revolver.task_queue.avail(), 1);
}

TEST_CASE("revolver.serve_empty")
{
    cnc::revolver revolver;
    robo::stepper steppers[1];
    robo::stepper *steppers_ptrs[] = {&steppers[0]};

    revolver.set_steppers(steppers_ptrs, 1);

    // serve on empty queue should not crash
    revolver.serve();
    revolver.serve();
    revolver.serve();

    CHECK(revolver.is_empty());
    CHECK_EQ(steppers[0].steps_count(), 0);
}

TEST_CASE("revolver.dda_positive_steps")
{
    cnc::revolver revolver;
    robo::stepper steppers[1];
    robo::stepper *steppers_ptrs[] = {&steppers[0]};

    revolver.set_steppers(steppers_ptrs, 1);

    // Acceleration 0.5 steps/tick² for 20 ticks
    // After 20 ticks: velocity = 10 steps/tick
    // Total distance ≈ sum of velocities ≈ 0.5 + 1 + 1.5 + ... + 10 = 105 steps
    std::array<cnc_float_type, NMAX_AXES> accels = {};
    accels[0] = 0.5;  // steps/tick²
    revolver.task_queue.emplace(accels, 20);

    // Then continue with zero acceleration (constant velocity)
    accels[0] = 0.0;
    revolver.task_queue.emplace(accels, 20);

    // Execute all ticks
    for (int i = 0; i < 40; ++i)
    {
        revolver.serve();
    }

    // Should have positive steps
    CHECK_GT(steppers[0].steps_count(), 0);
    CHECK(revolver.is_empty());
}

TEST_CASE("revolver.dda_negative_direction")
{
    cnc::revolver revolver;
    robo::stepper steppers[1];
    robo::stepper *steppers_ptrs[] = {&steppers[0]};

    revolver.set_steppers(steppers_ptrs, 1);

    // Negative acceleration -> negative velocity -> steps in minus direction
    std::array<cnc_float_type, NMAX_AXES> accels = {};
    accels[0] = -0.5;
    revolver.task_queue.emplace(accels, 20);

    accels[0] = 0.0;
    revolver.task_queue.emplace(accels, 20);

    for (int i = 0; i < 40; ++i)
    {
        revolver.serve();
    }

    // Steps should be negative
    CHECK_LT(steppers[0].steps_count(), 0);
}

TEST_CASE("revolver.multiaxis")
{
    cnc::revolver revolver;
    robo::stepper steppers[3];
    robo::stepper *steppers_ptrs[] = {&steppers[0], &steppers[1], &steppers[2]};

    revolver.set_steppers(steppers_ptrs, 3);

    // Different accelerations for different axes
    std::array<cnc_float_type, NMAX_AXES> accels = {};
    accels[0] = 0.5;   // positive
    accels[1] = -0.5;  // negative
    accels[2] = 0.25;  // smaller positive

    revolver.task_queue.emplace(accels, 20);

    accels[0] = 0.0;
    accels[1] = 0.0;
    accels[2] = 0.0;
    revolver.task_queue.emplace(accels, 20);

    for (int i = 0; i < 40; ++i)
    {
        revolver.serve();
    }

    // Axis 0 - positive steps
    CHECK_GT(steppers[0].steps_count(), 0);
    // Axis 1 - negative steps
    CHECK_LT(steppers[1].steps_count(), 0);
    // Axis 2 - positive, but less than axis 0
    CHECK_GT(steppers[2].steps_count(), 0);
    CHECK_LT(steppers[2].steps_count(), steppers[0].steps_count());
}

TEST_CASE("revolver.clear")
{
    cnc::revolver revolver;
    robo::stepper steppers[2];
    robo::stepper *steppers_ptrs[] = {&steppers[0], &steppers[1]};

    revolver.set_steppers(steppers_ptrs, 2);

    // Add tasks
    std::array<cnc_float_type, NMAX_AXES> accels = {};
    accels[0] = 0.3;
    revolver.task_queue.emplace(accels, 10);
    revolver.task_queue.emplace(accels, 10);

    CHECK_FALSE(revolver.is_empty());

    // Execute a few ticks
    revolver.serve();
    revolver.serve();

    // Clear
    revolver.clear();

    CHECK(revolver.is_empty());
    CHECK_EQ(revolver.current_task, nullptr);

    // Velocities should be reset
    auto velocities = revolver.get_current_velocities();
    CHECK_EQ(velocities[0], 0.0);
    CHECK_EQ(velocities[1], 0.0);
}

TEST_CASE("revolver.current_velocities")
{
    cnc::revolver revolver;
    robo::stepper steppers[2];
    robo::stepper *steppers_ptrs[] = {&steppers[0], &steppers[1]};

    revolver.set_steppers(steppers_ptrs, 2);

    // Initial velocities are zero
    auto vel0 = revolver.get_current_velocities();
    CHECK_EQ(vel0[0], 0.0);
    CHECK_EQ(vel0[1], 0.0);

    // Add task with acceleration
    std::array<cnc_float_type, NMAX_AXES> accels = {};
    accels[0] = 0.5;
    accels[1] = 0.25;
    revolver.task_queue.emplace(accels, 4);

    // After each serve, velocity increases by acceleration
    revolver.serve();
    auto vel1 = revolver.get_current_velocities();
    CHECK_EQ(vel1[0], doctest::Approx(0.5));
    CHECK_EQ(vel1[1], doctest::Approx(0.25));

    revolver.serve();
    auto vel2 = revolver.get_current_velocities();
    CHECK_EQ(vel2[0], doctest::Approx(1.0));
    CHECK_EQ(vel2[1], doctest::Approx(0.5));
}

TEST_CASE("revolver.current_steps")
{
    cnc::revolver revolver;
    robo::stepper steppers[2];
    robo::stepper *steppers_ptrs[] = {&steppers[0], &steppers[1]};

    revolver.set_steppers(steppers_ptrs, 2);

    // Initial steps are zero
    steps_t steps[2];
    revolver.get_current_steps(steps);
    CHECK_EQ(steps[0], 0);
    CHECK_EQ(steps[1], 0);

    // Add task with acceleration sufficient for several steps
    std::array<cnc_float_type, NMAX_AXES> accels = {};
    accels[0] = 1.0;  // 1 step/tick² - will accumulate fast
    accels[1] = 0.5;
    revolver.task_queue.emplace(accels, 20);

    accels[0] = 0.0;
    accels[1] = 0.0;
    revolver.task_queue.emplace(accels, 20);

    for (int i = 0; i < 40; ++i)
    {
        revolver.serve();
    }

    revolver.get_current_steps(steps);
    CHECK_GT(steps[0], 0);
    CHECK_GT(steps[1], 0);
    // Axis 0 should have more steps due to higher acceleration
    CHECK_GT(steps[0], steps[1]);
}

TEST_CASE("revolver.reset")
{
    cnc::revolver revolver;
    robo::stepper steppers[1];
    robo::stepper *steppers_ptrs[] = {&steppers[0]};

    revolver.set_steppers(steppers_ptrs, 1);

    // Add and execute tasks
    std::array<cnc_float_type, NMAX_AXES> accels = {};
    accels[0] = 0.5;
    revolver.task_queue.emplace(accels, 10);

    accels[0] = 0.0;
    revolver.task_queue.emplace(accels, 5);

    for (int i = 0; i < 15; ++i)
    {
        revolver.serve();
    }

    CHECK_GT(steppers[0].steps_count(), 0);

    // reset clears state
    revolver.reset();

    CHECK(revolver.is_empty());
    auto vel = revolver.get_current_velocities();
    CHECK_EQ(vel[0], 0.0);
}
