
#include <chrono>
#include <doctest/doctest.h>
#include <iomanip>
#include <limits>
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
    cnc_float_type control_scale = 1000.0;  // Same as robot_arm.json (pulses_per_unit)

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

        // Set control_scale like emulator does
        for (int i = 0; i < 4; ++i)
        {
            interpreter.set_control_scale(i, control_scale);
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

    // Expected: 10 units * 1000 pulses/unit = 10000 pulses
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
    cnc_float_type expected_max = 50.0 * fix.control_scale / fix.freq;

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
        int64_t expected_steps = static_cast<int64_t>(target * fix.control_scale);

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
    cnc_float_type expected_vel = 50.0 * fix.control_scale / fix.freq;
    cnc_float_type expected_acc = 200.0 * fix.control_scale / (fix.freq * fix.freq);

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

TEST_CASE("emulator-like: INTEGRATION ERROR diagnostic")
{
    // Analyze where the 2-step error comes from
    EmulatorTestFixture fix;
    fix.setup();

    fix.interpreter.newline("absmove Y1 F1 M1");
    auto block = fix.interpreter.last_block();

    MESSAGE("=== Block parameters ===");
    MESSAGE("axdist[1] = " << block.axdist[1]);
    MESSAGE("fullpath = " << block.fullpath);
    MESSAGE("nominal_velocity = " << std::setprecision(15) << block.nominal_velocity);
    MESSAGE("acceleration = " << std::setprecision(15) << block.acceleration);
    MESSAGE("acceleration_before_ic = " << block.acceleration_before_ic);
    MESSAGE("deceleration_after_ic = " << block.deceleration_after_ic);
    MESSAGE("block_finish_ic = " << block.block_finish_ic);

    // Calculate theoretical path
    // For triangle: path = 0.5 * a * t_accel² + 0.5 * a * t_decel²
    //             = a * t² (when t_accel = t_decel = t)
    int64_t t = block.acceleration_before_ic;  // half-time for triangle
    cnc_float_type theoretical_path = block.acceleration * t * t * 2;  // both phases
    MESSAGE("theoretical_path from a*t² = " << theoretical_path);

    // Also check: path = v_max * t (area of triangle = 0.5 * base * height * 2 triangles)
    cnc_float_type path_from_velocity = block.nominal_velocity * t;
    MESSAGE("path_from_velocity = v*t = " << path_from_velocity);

    // Fixed-point representation
    int64_t acc_fixed = static_cast<int64_t>(block.acceleration * FIXED_POINT_MUL);
    int64_t vel_fixed_max = static_cast<int64_t>(block.nominal_velocity * FIXED_POINT_MUL);
    MESSAGE("acc_fixed = " << acc_fixed);
    MESSAGE("vel_fixed_max = " << vel_fixed_max);

    // Simulate integration manually for first few ticks
    MESSAGE("=== Manual integration check ===");
    int64_t pos = 0;
    int64_t vel = 0;
    int64_t total_ticks = block.block_finish_ic;

    // Accel phase
    for (int64_t tick = 0; tick < t; ++tick)
    {
        int64_t half_acc = acc_fixed >> 1;
        pos += vel + half_acc;
        vel += acc_fixed;
    }
    MESSAGE("After accel phase: pos_fixed=" << pos << ", vel_fixed=" << vel);
    MESSAGE("  pos in steps = " << (double)pos / FIXED_POINT_MUL);
    MESSAGE("  vel in steps/tick = " << (double)vel / FIXED_POINT_MUL);

    // Decel phase
    for (int64_t tick = 0; tick < t; ++tick)
    {
        int64_t half_acc = (-acc_fixed) >> 1;
        pos += vel + half_acc;
        vel -= acc_fixed;
    }
    MESSAGE("After decel phase: pos_fixed=" << pos << ", vel_fixed=" << vel);
    MESSAGE("  pos in steps = " << (double)pos / FIXED_POINT_MUL);
    MESSAGE("  vel in steps/tick = " << (double)vel / FIXED_POINT_MUL);

    // Expected vs actual
    double final_pos_steps = (double)pos / FIXED_POINT_MUL;
    MESSAGE("Expected: 1000 steps");
    MESSAGE("Calculated: " << final_pos_steps << " steps");
    MESSAGE("Error: " << (1000 - final_pos_steps) << " steps");

    // Run actual simulation
    fix.run_for(30000ms);
    int64_t actual_steps = fix.get_steps(1);
    MESSAGE("Actual from simulation: " << actual_steps << " steps");

    CHECK_GE(actual_steps, 999);
    CHECK_LE(actual_steps, 1001);
}

TEST_CASE("emulator-like: OVERFLOW LIMITS analysis")
{
    // Analyze maximum values before int64_t overflow with FIXED_POINT_BITS=40
    //
    // int64_t max = 2^63 - 1 ≈ 9.22 × 10^18
    // FIXED_POINT_MUL = 2^40 ≈ 1.1 × 10^12

    constexpr int64_t INT64_MAX_VAL = std::numeric_limits<int64_t>::max();
    constexpr double freq = 100000.0;  // 100 kHz
    constexpr double steps_per_unit = 1000.0;

    MESSAGE("=== OVERFLOW LIMITS with FIXED_POINT_BITS = " << FIXED_POINT_BITS << " ===");
    MESSAGE("FIXED_POINT_MUL = " << FIXED_POINT_MUL);
    MESSAGE("int64_t max = " << INT64_MAX_VAL);
    MESSAGE("");

    // 1. Maximum velocity in fixed-point
    // velocities_fixed must fit in int64_t
    // vel_fixed = vel_steps_per_tick * FIXED_POINT_MUL < INT64_MAX
    // vel_steps_per_tick < INT64_MAX / FIXED_POINT_MUL = 2^63 / 2^40 = 2^23
    double max_vel_steps_per_tick = (double)INT64_MAX_VAL / FIXED_POINT_MUL;
    double max_vel_units_per_sec = max_vel_steps_per_tick * freq / steps_per_unit;

    MESSAGE("=== VELOCITY LIMITS ===");
    MESSAGE("Max velocity: " << max_vel_steps_per_tick << " steps/tick");
    MESSAGE("Max velocity: " << max_vel_units_per_sec << " units/sec");
    MESSAGE("  (at freq=" << freq << " Hz, steps_per_unit=" << steps_per_unit << ")");
    MESSAGE("");

    // 2. Maximum acceleration in fixed-point
    // Same logic: acc_fixed < INT64_MAX
    double max_acc_steps_per_tick2 = (double)INT64_MAX_VAL / FIXED_POINT_MUL;
    double max_acc_units_per_sec2 = max_acc_steps_per_tick2 * freq * freq / steps_per_unit;

    MESSAGE("=== ACCELERATION LIMITS ===");
    MESSAGE("Max acceleration: " << max_acc_steps_per_tick2 << " steps/tick²");
    MESSAGE("Max acceleration: " << max_acc_units_per_sec2 << " units/sec²");
    MESSAGE("");

    // 3. Position accumulation risk
    // positions_fixed resets when crossing integer step boundaries
    // So positions_fixed is always in range [0, FIXED_POINT_MUL) or (-FIXED_POINT_MUL, 0]
    // No overflow risk for position itself

    MESSAGE("=== POSITION ===");
    MESSAGE("positions_fixed always < FIXED_POINT_MUL (resets at each step)");
    MESSAGE("No overflow risk for position accumulation");
    MESSAGE("");

    // 4. Practical limits for robot arm config
    MESSAGE("=== PRACTICAL LIMITS for robot_arm.json config ===");
    MESSAGE("freq=100kHz, steps_per_unit=1000:");
    MESSAGE("  Max velocity: " << max_vel_units_per_sec << " rad/s");
    MESSAGE("    (typical robot: 1-10 rad/s, margin: " << max_vel_units_per_sec / 10 << "x)");
    MESSAGE("  Max acceleration: " << max_acc_units_per_sec2 << " rad/s²");
    MESSAGE("    (typical robot: 1-100 rad/s², margin: " << max_acc_units_per_sec2 / 100 << "x)");

    // Verify we have huge safety margins
    CHECK_GT(max_vel_units_per_sec, 1e6);   // > 1 million units/s
    CHECK_GT(max_acc_units_per_sec2, 1e10); // > 10 billion units/s²

    // === HIGH RESOLUTION CASE ===
    // steps_per_unit = 1 000 000 (high-res encoder)
    // velocity = 100 units/s
    MESSAGE("");
    MESSAGE("=== HIGH RESOLUTION CASE (steps_per_unit=1000000) ===");
    constexpr double high_res_steps_per_unit = 1000000.0;
    constexpr double typical_velocity = 100.0;  // units/s

    double vel_steps_per_tick = typical_velocity * high_res_steps_per_unit / freq;
    double vel_fixed_value = vel_steps_per_tick * FIXED_POINT_MUL;

    MESSAGE("At velocity=" << typical_velocity << " units/s:");
    MESSAGE("  vel_steps_per_tick = " << vel_steps_per_tick);
    MESSAGE("  vel_fixed = " << std::scientific << vel_fixed_value);
    MESSAGE("  int64_t max = " << (double)INT64_MAX_VAL);
    MESSAGE("  margin = " << (double)INT64_MAX_VAL / vel_fixed_value << "x");

    double max_vel_high_res = max_vel_steps_per_tick * freq / high_res_steps_per_unit;
    MESSAGE("  Max velocity before overflow: " << max_vel_high_res << " units/s");
    MESSAGE("");

    // WARNING: at 1000 steps/tick, the while loop in serve() runs 1000 times!
    MESSAGE("=== PERFORMANCE WARNING ===");
    MESSAGE("At " << vel_steps_per_tick << " steps/tick:");
    MESSAGE("  The step generation loop runs ~" << (int)vel_steps_per_tick << " iterations per serve() call");
    MESSAGE("  At 100kHz this may be too slow for real-time!");
    MESSAGE("  Consider reducing steps_per_unit or frequency for high-speed motion");

    // This config still works but with reduced margin
    CHECK_GT(max_vel_high_res, typical_velocity);  // Must be able to do 100 units/s
}

TEST_CASE("emulator-like: FIXED-POINT PRECISION diagnostic")
{
    // After fix: FIXED_POINT_MUL = 2^40
    // For M=1: acceleration = 1 unit/s² * 1000 steps/unit / (100000 Hz)² = 1e-7 steps/tick²
    // acceleration_fixed = 1e-7 * 2^40 = 109951 -> ~0.0004% error (excellent!)

    EmulatorTestFixture fix;
    fix.setup();

    MESSAGE("=== Fixed-point precision analysis ===");
    MESSAGE("FIXED_POINT_MUL = " << FIXED_POINT_MUL);
    MESSAGE("freq = " << fix.freq << " Hz");
    MESSAGE("steps_per_unit = " << fix.control_scale);

    std::vector<double> m_values = {0.1, 0.5, 1, 2, 5, 10, 50, 100, 500, 1000};

    for (double m_val : m_values)
    {
        // Calculate expected acceleration in steps/tick²
        cnc_float_type acc_steps_per_tick2 = m_val * fix.control_scale / (fix.freq * fix.freq);

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
    double min_M_for_1pct = 100.0 * fix.freq * fix.freq / (fix.control_scale * FIXED_POINT_MUL);
    MESSAGE("Minimum M for 1% precision = " << min_M_for_1pct << " units/s²");

    // For robot arm with max_acceleration = 3.14 rad/s², this is a problem!
    CHECK_LT(min_M_for_1pct, 10.0);  // Should be able to use M < 10 with good precision
}

TEST_CASE("emulator-like: smooth_stop decelerates correctly")
{
    EmulatorTestFixture fix;
    fix.setup();

    // Start a long move
    fix.interpreter.newline("absmove Y100 F10 M10");
    auto block = fix.interpreter.last_block();

    MESSAGE("=== Starting long move ===");
    MESSAGE("target = 100 units");
    MESSAGE("expected_steps = " << 100.0 * fix.control_scale);
    MESSAGE("nominal_velocity = " << block.nominal_velocity);
    MESSAGE("acceleration = " << block.acceleration);

    // Execute only 20% of the motion
    int ticks_20_pct = block.block_finish_ic / 5;
    MESSAGE("block_finish_ic = " << block.block_finish_ic);
    MESSAGE("executing " << ticks_20_pct << " ticks (20%)");

    for (int i = 0; i < ticks_20_pct; ++i)
    {
        fix.planner.serve();
        fix.revolver.serve();
    }

    auto steps_before_stop = fix.revolver.current_steps();
    auto vels_before_stop = fix.revolver.current_velocities();
    MESSAGE("steps before stop: " << steps_before_stop[1]);
    MESSAGE("velocity before stop: " << vels_before_stop[1] << " steps/tick");

    // Call stop command
    MESSAGE("=== Calling stop ===");
    fix.interpreter.newline("stop");

    // Get the stop block parameters
    auto stop_block = fix.interpreter.last_block();
    MESSAGE("stop_block.nominal_velocity = " << stop_block.nominal_velocity);
    MESSAGE("stop_block.acceleration = " << stop_block.acceleration);
    MESSAGE("stop_block.block_finish_ic = " << stop_block.block_finish_ic);
    MESSAGE("stop_block.start_velocity = " << stop_block.start_velocity);
    MESSAGE("stop_block.final_velocity = " << stop_block.final_velocity);

    // The stop block should have reasonable deceleration (not zero or extremely small)
    // With the fix, acceleration should be in steps/tick² (same units as velocity)
    CHECK_GT(stop_block.acceleration, 1e-10);  // Not too small
    CHECK_LT(stop_block.block_finish_ic, 10000000);  // Not 100 seconds!

    // Execute until stopped
    int max_ticks = stop_block.block_finish_ic + 10000;
    for (int i = 0; i < max_ticks; ++i)
    {
        fix.planner.serve();
        fix.revolver.serve();
    }

    auto vels_after_stop = fix.revolver.current_velocities();
    MESSAGE("velocity after stop: " << vels_after_stop[1] << " steps/tick");

    // Velocity should be near zero (stopped)
    CHECK_LT(std::abs(vels_after_stop[1]), 1e-6);
}

TEST_CASE("emulator-like: smooth_stop unit conversion")
{
    // This test specifically verifies the unit conversion fix in smooth_stop()
    // The bug was: external_acceleration was in units/tick² but velocity was in steps/tick

    EmulatorTestFixture fix;
    fix.setup();

    MESSAGE("=== Unit conversion test for smooth_stop ===");
    MESSAGE("freq = " << fix.freq);
    MESSAGE("steps_per_unit = " << fix.control_scale);

    // Start a move and get to cruising velocity
    fix.interpreter.newline("absmove Y100 F10 M10");
    auto block = fix.interpreter.last_block();

    // Execute until we're in the flat (cruising) phase
    int ticks_to_cruise = block.acceleration_before_ic + 100;
    MESSAGE("ticks_to_cruise = " << ticks_to_cruise);

    for (int i = 0; i < ticks_to_cruise; ++i)
    {
        fix.planner.serve();
        fix.revolver.serve();
    }

    auto vels_cruising = fix.revolver.current_velocities();
    double vel_steps_per_tick = vels_cruising[1];
    MESSAGE("cruising velocity = " << vel_steps_per_tick << " steps/tick");

    // Expected deceleration time if units are correct:
    // v = F * steps_per_unit / freq = 10 * 1000 / 100000 = 0.1 steps/tick
    // a = M * steps_per_unit / freq² = 10 * 1000 / 1e10 = 1e-6 steps/tick²
    // t_stop = v / a = 0.1 / 1e-6 = 100000 ticks = 1 second

    double expected_vel = 10.0 * fix.control_scale / fix.freq;
    double expected_acc = 10.0 * fix.control_scale / (fix.freq * fix.freq);
    double expected_stop_ticks = expected_vel / expected_acc;

    MESSAGE("expected velocity = " << expected_vel << " steps/tick");
    MESSAGE("expected acceleration = " << expected_acc << " steps/tick²");
    MESSAGE("expected stop time = " << expected_stop_ticks << " ticks");

    // Call stop
    fix.interpreter.newline("stop");
    auto stop_block = fix.interpreter.last_block();

    MESSAGE("stop_block.start_velocity = " << stop_block.start_velocity);
    MESSAGE("stop_block.acceleration = " << stop_block.acceleration);
    MESSAGE("stop_block.block_finish_ic = " << stop_block.block_finish_ic);

    // Verify the stop block has correct units
    // The deceleration time should be on the order of expected_stop_ticks, not 1000x longer
    double actual_stop_ticks = stop_block.block_finish_ic;
    double ratio = actual_stop_ticks / expected_stop_ticks;

    MESSAGE("actual stop time = " << actual_stop_ticks << " ticks");
    MESSAGE("ratio actual/expected = " << ratio);

    // The ratio should be close to 1 (within 2x is acceptable due to rounding)
    CHECK_LT(ratio, 5.0);
    CHECK_GT(ratio, 0.2);
}
