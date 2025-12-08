
#include <chrono>
#include <doctest/doctest.h>
#include <mutex>
#include <thread>

#include <ralgo/cnc/interpreter.h>
#include <ralgo/cnc/planner.h>
#include <ralgo/cnc/revolver.h>
#include <ralgo/freq_invoke_imitation.h>
#include <ralgo/global_protection.h>

nos::log::logger emulator_logger;
using namespace std::chrono_literals;

// Test setup similar to ralgo-cnc-emulator configuration
class EmulatorTestFixture
{
public:
    igris::ring<cnc::planner_block> blocks{100};
    cnc::revolver revolver;
    cnc::planner planner{&blocks, &revolver};
    cnc::feedback_guard feedback_guard{4};
    cnc::interpreter interpreter{&blocks, &planner, &revolver, &feedback_guard};
    robo::stepper steppers[4];
    robo::stepper *steppers_ptrs[4] = {&steppers[0], &steppers[1], &steppers[2], &steppers[3]};

    cnc_float_type freq = 100000.0;  // Same as emulator config
    cnc_float_type steps_per_unit = 1000.0;  // Same as robot_arm.json

    ralgo::freq_invoke_imitation imitator = {
        {1ms, [this]() { planner.serve(); }},
        {10us, [this]() { revolver.serve(); }},  // 100kHz
    };

    void setup()
    {
        ralgo::set_logger(&emulator_logger);
        ralgo::global_protection = false;

        interpreter.init_axes(4);
        interpreter.set_revolver_frequency(freq);
        revolver.set_steppers(steppers_ptrs, 4);

        // Set gears like emulator does
        for (int i = 0; i < 4; ++i)
        {
            interpreter.set_steps_per_unit(i, steps_per_unit);
        }
    }

    void run_for(std::chrono::milliseconds duration)
    {
        imitator.timeskip(duration);
    }

    int64_t get_steps(int axis)
    {
        return steppers[axis].steps();
    }

    double get_position(int axis)
    {
        return interpreter.final_position()[axis];
    }
};

TEST_CASE("emulator-like: args_to_index_value_map diagnostic")
{
    // Direct test of the parsing
    auto tokens = nos::tokens("velmaxs 0:100 1:200");
    nos::argv argv(tokens);

    MESSAGE("tokens.size() = " << tokens.size());
    for (size_t i = 0; i < tokens.size(); ++i)
        MESSAGE("tokens[" << i << "] = '" << tokens[i] << "'");

    MESSAGE("argv.size() = " << argv.size());
    for (size_t i = 0; i < argv.size(); ++i)
        MESSAGE("argv[" << i << "] = '" << argv[i] << "'");

    auto without1 = argv.without(1);
    MESSAGE("argv.without(1).size() = " << without1.size());
    for (size_t i = 0; i < without1.size(); ++i)
        MESSAGE("without1[" << i << "] = '" << without1[i] << "'");

    auto fmap = args_to_index_value_map(without1);
    MESSAGE("fmap.size() = " << fmap.size());
    for (auto& [k, v] : fmap)
        MESSAGE("fmap[" << k << "] = " << v);

    CHECK_EQ(fmap.size(), 2);
    CHECK_EQ(fmap[0], 100);
    CHECK_EQ(fmap[1], 200);
}

TEST_CASE("emulator-like: cmd prefix velmaxs works")
{
    EmulatorTestFixture fix;
    fix.setup();

    // Test with "cmd" prefix like emulator does
    auto result = fix.interpreter.newline("cmd velmaxs 0:100 1:100 2:100 3:100");
    MESSAGE("cmd velmaxs result: '" << result << "'");

    // Verify by checking state output
    auto state = fix.interpreter.newline("state");
    MESSAGE("state after cmd velmaxs: " << state);

    // Should contain velocity limits
    CHECK(state.find("velmaxs:{100") != std::string::npos);
}

TEST_CASE("emulator-like: velmaxs without prefix works")
{
    EmulatorTestFixture fix;
    fix.setup();

    // Test without prefix (new style)
    auto result = fix.interpreter.newline("velmaxs 0:100 1:100 2:100 3:100");
    MESSAGE("velmaxs result: '" << result << "'");

    // Verify by checking state output
    auto state = fix.interpreter.newline("state");
    MESSAGE("state after velmaxs: " << state);

    CHECK(state.find("100") != std::string::npos);
}

TEST_CASE("emulator-like: basic move reaches target")
{
    EmulatorTestFixture fix;
    fix.setup();

    // Move Y axis 10 units (should be 10000 steps)
    auto response = fix.interpreter.newline("absmove Y10 F100 M500");
    CHECK_EQ(response, "");

    auto last_block = fix.interpreter.last_block();

    MESSAGE("axdist[1] = " << last_block.axdist[1]);
    MESSAGE("fullpath = " << last_block.fullpath);
    MESSAGE("nominal_velocity = " << last_block.nominal_velocity);
    MESSAGE("acceleration = " << last_block.acceleration);

    // Expected: 10 units * 1000 steps/unit = 10000 steps
    CHECK_EQ(last_block.axdist[1], 10000);
    CHECK_EQ(last_block.fullpath, 10000);

    // Run simulation for enough time
    fix.run_for(5000ms);

    int64_t final_steps = fix.get_steps(1);
    double final_pos = fix.get_position(1);

    MESSAGE("final_steps = " << final_steps);
    MESSAGE("final_pos = " << final_pos);

    // Should reach target within 0.1%
    CHECK_GE(final_steps, 9990);
    CHECK_LE(final_steps, 10010);
    CHECK_GE(final_pos, 9.99);
    CHECK_LE(final_pos, 10.01);
}

TEST_CASE("emulator-like: F parameter affects velocity")
{
    EmulatorTestFixture fix;
    fix.setup();

    // Test with low feed
    fix.interpreter.newline("absmove Y10 F10 M500");
    auto block_slow = fix.interpreter.last_block();

    // Reset position
    fix.run_for(10000ms);
    fix.interpreter.newline("absmove Y0 F100 M500");
    fix.run_for(10000ms);

    // Test with high feed
    fix.interpreter.newline("absmove Y10 F100 M500");
    auto block_fast = fix.interpreter.last_block();

    MESSAGE("slow nominal_velocity = " << block_slow.nominal_velocity);
    MESSAGE("fast nominal_velocity = " << block_fast.nominal_velocity);

    // Fast should have higher velocity
    CHECK_GT(block_fast.nominal_velocity, block_slow.nominal_velocity);

    // Velocity ratio should be approximately 10x
    double ratio = block_fast.nominal_velocity / block_slow.nominal_velocity;
    MESSAGE("velocity ratio = " << ratio);
    CHECK_GT(ratio, 5.0);  // At least 5x faster
}

TEST_CASE("emulator-like: M parameter affects acceleration")
{
    EmulatorTestFixture fix;
    fix.setup();

    // Test with low acceleration
    fix.interpreter.newline("absmove Y10 F100 M50");
    auto block_slow = fix.interpreter.last_block();

    // Reset
    fix.run_for(10000ms);
    fix.interpreter.newline("absmove Y0 F100 M500");
    fix.run_for(10000ms);

    // Test with high acceleration
    fix.interpreter.newline("absmove Y10 F100 M500");
    auto block_fast = fix.interpreter.last_block();

    MESSAGE("slow acceleration = " << block_slow.acceleration);
    MESSAGE("fast acceleration = " << block_fast.acceleration);

    // Fast should have higher acceleration
    CHECK_GT(block_fast.acceleration, block_slow.acceleration);

    // Acceleration ratio should be approximately 10x
    double ratio = block_fast.acceleration / block_slow.acceleration;
    MESSAGE("acceleration ratio = " << ratio);
    CHECK_GT(ratio, 5.0);
}

TEST_CASE("emulator-like: velmaxs limits velocity")
{
    EmulatorTestFixture fix;
    fix.setup();

    // Set velocity limit on Y axis
    fix.interpreter.newline("velmaxs 1:50");  // 50 units/sec max

    // Request faster than limit
    fix.interpreter.newline("absmove Y10 F100 M500");
    auto block = fix.interpreter.last_block();

    // Velocity should be limited
    // Expected: 50 units/s * 1000 steps/unit / 100000 Hz = 0.5 steps/tick
    cnc_float_type expected_max = 50.0 * fix.steps_per_unit / fix.freq;

    MESSAGE("nominal_velocity = " << block.nominal_velocity);
    MESSAGE("expected_max = " << expected_max);

    CHECK_LE(block.nominal_velocity, expected_max * 1.01);  // Allow 1% tolerance
}

TEST_CASE("emulator-like: position accuracy with high gear ratio")
{
    EmulatorTestFixture fix;
    fix.setup();

    // Multiple moves to check accumulated error
    std::vector<double> targets = {10, 0, 5, 15, 7.5, 0};

    for (double target : targets)
    {
        std::string cmd = "absmove Y" + std::to_string(target) + " F100 M500";
        fix.interpreter.newline(cmd);
        fix.run_for(5000ms);

        double pos = fix.get_position(1);
        int64_t steps = fix.get_steps(1);
        int64_t expected_steps = static_cast<int64_t>(target * fix.steps_per_unit);

        MESSAGE("target = " << target);
        MESSAGE("position = " << pos);
        MESSAGE("steps = " << steps);
        MESSAGE("expected_steps = " << expected_steps);

        // Should be within 0.5%
        CHECK(std::abs(pos - target) < target * 0.005 + 0.01);
    }
}

TEST_CASE("emulator-like: check F/M parsing without prefixes")
{
    EmulatorTestFixture fix;
    fix.setup();

    // Set high limits so they don't interfere
    fix.interpreter.newline("velmaxs 0:1000 1:1000 2:1000 3:1000");
    fix.interpreter.newline("accmaxs 0:10000 1:10000 2:10000 3:10000");

    // Test that F and M are parsed correctly from command line
    // Use longer path (100 units) to ensure trapezoidal profile, not triangular
    // For F=50, M=200: accel_time = 50/200 = 0.25s, accel_dist = 0.5*200*0.25² = 6.25 units
    // So 100 units is plenty for trapezoid (>= 2*6.25 = 12.5 units needed)
    fix.interpreter.newline("relmove Y100 F50 M200");
    auto block = fix.interpreter.last_block();

    // F=50 units/s, M=200 units/s^2
    // Convert to steps/tick and steps/tick^2
    cnc_float_type expected_vel = 50.0 * fix.steps_per_unit / fix.freq;
    cnc_float_type expected_acc = 200.0 * fix.steps_per_unit / (fix.freq * fix.freq);

    MESSAGE("block.nominal_velocity = " << block.nominal_velocity);
    MESSAGE("expected_vel = " << expected_vel);
    MESSAGE("block.acceleration = " << block.acceleration);
    MESSAGE("expected_acc = " << expected_acc);

    // Allow 10% tolerance for rounding
    CHECK(std::abs(block.nominal_velocity - expected_vel) < expected_vel * 0.1);
    CHECK(std::abs(block.acceleration - expected_acc) < expected_acc * 0.1);
}

TEST_CASE("emulator-like: LOW M parameter - position error diagnostic")
{
    // Test M=1 specifically - should work now with 32-bit fixed point
    EmulatorTestFixture fix;
    fix.setup();

    fix.interpreter.newline("absmove Y1 F1 M1");
    auto block = fix.interpreter.last_block();

    MESSAGE("=== M=1 test ===");
    MESSAGE("block.axdist[1] = " << block.axdist[1]);
    MESSAGE("block.fullpath = " << block.fullpath);
    MESSAGE("block.nominal_velocity = " << block.nominal_velocity);
    MESSAGE("block.acceleration = " << block.acceleration);
    MESSAGE("block.acceleration_before_ic = " << block.acceleration_before_ic);
    MESSAGE("block.deceleration_after_ic = " << block.deceleration_after_ic);
    MESSAGE("block.block_finish_ic = " << block.block_finish_ic);

    fix.run_for(30000ms);

    int64_t final_steps = fix.get_steps(1);
    MESSAGE("final_steps = " << final_steps << " (expected 1000)");
    MESSAGE("error = " << std::abs(1000 - final_steps) << " steps ("
            << (100.0 * std::abs(1000 - final_steps) / 1000.0) << "%)");

    // Expected: 1 unit * 1000 steps/unit = 1000 steps
    CHECK_EQ(block.axdist[1], 1000);
    CHECK_GE(final_steps, 990);  // Allow 1% error
    CHECK_LE(final_steps, 1010);
}

TEST_CASE("emulator-like: LOW M=0.5 position test")
{
    EmulatorTestFixture fix;
    fix.setup();

    fix.interpreter.newline("absmove Y1 F1 M0.5");
    auto block = fix.interpreter.last_block();

    MESSAGE("=== M=0.5 test ===");
    MESSAGE("block.axdist[1] = " << block.axdist[1]);
    MESSAGE("block.nominal_velocity = " << block.nominal_velocity);
    MESSAGE("block.acceleration = " << block.acceleration);

    // Run long enough (triangular profile with very slow accel)
    fix.run_for(60000ms);

    int64_t final_steps = fix.get_steps(1);
    MESSAGE("final_steps = " << final_steps << " (expected 1000)");
    MESSAGE("error = " << std::abs(1000 - final_steps) << " steps ("
            << (100.0 * std::abs(1000 - final_steps) / 1000.0) << "%)");

    CHECK_EQ(block.axdist[1], 1000);
    CHECK_GE(final_steps, 990);
    CHECK_LE(final_steps, 1010);
}

TEST_CASE("emulator-like: M < 1 should still work")
{
    EmulatorTestFixture fix;
    fix.setup();

    // M=0.5 should work - just slow acceleration
    fix.interpreter.newline("absmove Y1 F1 M0.5");
    auto block = fix.interpreter.last_block();

    MESSAGE("M=0.5 block:");
    MESSAGE("  axdist[1] = " << block.axdist[1]);
    MESSAGE("  fullpath = " << block.fullpath);
    MESSAGE("  nominal_velocity = " << block.nominal_velocity);
    MESSAGE("  acceleration = " << block.acceleration);

    // acceleration should be positive
    CHECK_GT(block.acceleration, 0);

    // axdist should be 1000 steps
    CHECK_EQ(block.axdist[1], 1000);

    // Run motion
    fix.run_for(60000ms);  // Extra long for slow acceleration

    int64_t final_steps = fix.get_steps(1);
    MESSAGE("final_steps = " << final_steps);

    CHECK_GE(final_steps, 990);
    CHECK_LE(final_steps, 1010);
}

TEST_CASE("emulator-like: FIXED-POINT PRECISION diagnostic")
{
    // After fix: FIXED_POINT_MUL = 2^32 = 4294967296
    // For M=1: acceleration = 1 unit/s² * 1000 steps/unit / (100000 Hz)² = 1e-7 steps/tick²
    // acceleration_fixed = 1e-7 * 2^32 = 430 -> ~0.2% error (good!)
    //
    // Before fix (2^24): acceleration_fixed = 1.68 -> 40% error!

    EmulatorTestFixture fix;
    fix.setup();

    MESSAGE("=== Fixed-point precision analysis ===");
    MESSAGE("FIXED_POINT_MUL = " << FIXED_POINT_MUL);
    MESSAGE("freq = " << fix.freq << " Hz");
    MESSAGE("steps_per_unit = " << fix.steps_per_unit);

    std::vector<double> m_values = {0.1, 0.5, 1, 2, 5, 10, 50, 100, 500, 1000};

    for (double m_val : m_values)
    {
        // Calculate expected acceleration in steps/tick²
        cnc_float_type acc_steps_per_tick2 = m_val * fix.steps_per_unit / (fix.freq * fix.freq);

        // What it becomes in fixed-point
        int64_t acc_fixed = static_cast<int64_t>(acc_steps_per_tick2 * FIXED_POINT_MUL);

        // Relative error from truncation
        double reconstructed = static_cast<double>(acc_fixed) / FIXED_POINT_MUL;
        double relative_error = (acc_steps_per_tick2 > 0) ?
            std::abs(reconstructed - acc_steps_per_tick2) / acc_steps_per_tick2 * 100.0 : 0;

        MESSAGE("M=" << m_val
            << " -> acc=" << acc_steps_per_tick2
            << " steps/tick², fixed=" << acc_fixed
            << ", error=" << relative_error << "%");

        if (m_val >= 1.0)
        {
            // For M >= 1, fixed-point representation should be non-zero
            CHECK_GT(acc_fixed, 0);
        }
    }

    MESSAGE("");
    MESSAGE("=== Minimum M for 1% precision ===");
    // For 1% precision, we need acc_fixed >= 100 (so truncation error < 1%)
    // acc_fixed = M * steps_per_unit / freq² * FIXED_POINT_MUL >= 100
    // M >= 100 * freq² / (steps_per_unit * FIXED_POINT_MUL)
    double min_M_for_1pct = 100.0 * fix.freq * fix.freq / (fix.steps_per_unit * FIXED_POINT_MUL);
    MESSAGE("Minimum M for 1% precision = " << min_M_for_1pct << " units/s²");

    // For robot arm with max_acceleration = 3.14 rad/s², this is a problem!
    CHECK_LT(min_M_for_1pct, 10.0);  // Should be able to use M < 10 with good precision
}
