#include <cmath>
#include <doctest/doctest.h>
#include <ralgo/cnc/interpreter.h>
#include <ralgo/cnc/planner.h>
#include <ralgo/cnc/revolver.h>
#include <ralgo/global_protection.h>

// Test fixture for spiral motion with realistic helix-cnc configuration
class SpiralLookaheadFixture
{
public:
    igris::ring<cnc::planner_block> blocks{100};
    cnc::revolver revolver;
    cnc::planner planner{&blocks, &revolver};
    cnc::error_handler errors;
    cnc::interpreter interpreter{&blocks, &planner, &revolver, nullptr, &errors};

    // Realistic helix-cnc parameters
    cnc_float_type freq = 100000.0;          // 100 kHz
    cnc_float_type control_scale = 51471.0;  // pulses per radian (typical for helix)
    cnc_float_type velocity = 1.0;           // rad/s
    cnc_float_type acceleration = 3.14;      // rad/s²
    cnc_float_type junction_deviation = 0.01; // 0.01 rad

    int total_axes = 2;

    void setup()
    {
        ralgo::global_protection = false;
        interpreter.init_axes(total_axes);
        interpreter.set_revolver_frequency(freq);

        for (int i = 0; i < total_axes; ++i)
        {
            interpreter.set_control_scale(i, control_scale);
        }

        // Set look-ahead
        interpreter.set_junction_deviation(junction_deviation);
    }

    // Generate spiral segments: x = r*cos(t), y = r*sin(t)
    // Returns list of (dx, dy) increments
    std::vector<std::pair<cnc_float_type, cnc_float_type>>
    generate_spiral(int num_segments, cnc_float_type total_angle, cnc_float_type radius)
    {
        std::vector<std::pair<cnc_float_type, cnc_float_type>> segments;
        cnc_float_type dt = total_angle / num_segments;

        cnc_float_type prev_x = radius;
        cnc_float_type prev_y = 0;

        for (int i = 1; i <= num_segments; ++i)
        {
            cnc_float_type t = dt * i;
            cnc_float_type x = radius * std::cos(t);
            cnc_float_type y = radius * std::sin(t);
            cnc_float_type dx = x - prev_x;
            cnc_float_type dy = y - prev_y;
            segments.push_back({dx, dy});
            prev_x = x;
            prev_y = y;
        }
        return segments;
    }

    // Add block directly (bypassing interpreter string parsing)
    void add_block(cnc_float_type dx, cnc_float_type dy)
    {
        // Convert user units to pulses
        cnc_float_type task_pulses[] = {dx * control_scale, dy * control_scale};
        ralgo::vector_view<cnc_float_type> task_view{task_pulses, 2};

        // Convert velocity/acceleration to pulses/tick
        cnc_float_type vel_pulses = velocity * control_scale / freq;
        cnc_float_type acc_pulses = acceleration * control_scale / (freq * freq);

        auto &block = blocks.head_place();
        block.set_state(task_view, total_axes, vel_pulses, acc_pulses);
        blocks.move_head_one();
    }

    // Check block validity
    bool is_block_valid(const cnc::planner_block &block)
    {
        if (std::isnan(block.fullpath) || std::isinf(block.fullpath))
            return false;
        if (block.fullpath < 0 || block.fullpath > 1e15)
            return false;
        if (std::isnan(block.nominal_velocity) || std::isinf(block.nominal_velocity))
            return false;
        if (std::isnan(block.acceleration) || std::isinf(block.acceleration))
            return false;
        if (block.block_finish_ic < 0 || block.block_finish_ic > 1000000000)
            return false;
        if (block.block_finish_ic < block.start_ic)
            return false;
        if (block.acceleration_before_ic < block.start_ic)
            return false;
        if (block.deceleration_after_ic < block.acceleration_before_ic)
            return false;
        if (block.block_finish_ic < block.deceleration_after_ic)
            return false;
        return true;
    }

    int count_errors()
    {
        int count = 0;
        cnc::error_entry entry;
        while (errors.pop(entry))
        {
            count++;
            MESSAGE("Error: code=" << static_cast<int>(entry.code)
                    << " ctx=" << entry.context);
        }
        return count;
    }
};

TEST_CASE("spiral_lookahead: basic spiral without lookahead")
{
    SpiralLookaheadFixture fix;
    fix.setup();
    fix.interpreter.set_junction_deviation(0);  // Disable look-ahead

    auto segments = fix.generate_spiral(36, 2 * M_PI, 0.1);  // Full circle, 10 deg per segment

    MESSAGE("=== Adding " << segments.size() << " spiral segments ===");

    for (size_t i = 0; i < segments.size(); ++i)
    {
        fix.add_block(segments[i].first, segments[i].second);
    }

    MESSAGE("Blocks in queue: " << fix.blocks.avail());

    // Check all blocks are valid
    int invalid_count = 0;
    for (size_t i = 0; i < fix.blocks.avail(); ++i)
    {
        auto &block = fix.blocks.get(fix.blocks.fixup_index(fix.blocks.tail_index() + i));
        if (!fix.is_block_valid(block))
        {
            invalid_count++;
            MESSAGE("Block " << i << " INVALID: fullpath=" << block.fullpath
                    << " block_finish_ic=" << block.block_finish_ic);
        }
    }

    CHECK_EQ(invalid_count, 0);
    CHECK_EQ(fix.count_errors(), 0);
}

TEST_CASE("spiral_lookahead: spiral with lookahead enabled")
{
    SpiralLookaheadFixture fix;
    fix.setup();

    auto segments = fix.generate_spiral(36, 2 * M_PI, 0.1);

    MESSAGE("=== Adding " << segments.size() << " spiral segments with look-ahead ===");
    MESSAGE("junction_deviation = " << fix.junction_deviation);

    for (size_t i = 0; i < segments.size(); ++i)
    {
        fix.add_block(segments[i].first, segments[i].second);
    }

    // Apply look-ahead
    cnc_float_type junction_deviation_pulses = fix.junction_deviation * fix.control_scale;
    fix.planner.recalculate_block_velocities(junction_deviation_pulses);

    MESSAGE("Blocks after look-ahead: " << fix.blocks.avail());

    // Check all blocks are valid
    int invalid_count = 0;
    for (size_t i = 0; i < fix.blocks.avail(); ++i)
    {
        auto &block = fix.blocks.get(fix.blocks.fixup_index(fix.blocks.tail_index() + i));
        if (!fix.is_block_valid(block))
        {
            invalid_count++;
            MESSAGE("Block " << i << " INVALID after look-ahead:");
            MESSAGE("  fullpath=" << block.fullpath);
            MESSAGE("  start_velocity=" << block.start_velocity);
            MESSAGE("  final_velocity=" << block.final_velocity);
            MESSAGE("  nominal_velocity=" << block.nominal_velocity);
            MESSAGE("  acceleration=" << block.acceleration);
            MESSAGE("  block_finish_ic=" << block.block_finish_ic);
        }
    }

    int error_count = fix.count_errors();
    CHECK_EQ(invalid_count, 0);
    CHECK_EQ(error_count, 0);
}

TEST_CASE("spiral_lookahead: many segments stress test")
{
    SpiralLookaheadFixture fix;
    fix.setup();

    // Many small segments - like real spiral motion
    int num_segments = 90;  // Queue holds 100, leave room
    auto segments = fix.generate_spiral(num_segments, 10 * M_PI, 0.05);  // 5 rotations

    MESSAGE("=== Stress test: " << segments.size() << " segments ===");

    for (size_t i = 0; i < segments.size(); ++i)
    {
        fix.add_block(segments[i].first, segments[i].second);
    }

    // Apply look-ahead
    cnc_float_type junction_deviation_pulses = fix.junction_deviation * fix.control_scale;
    fix.planner.recalculate_block_velocities(junction_deviation_pulses);

    // Check blocks
    int invalid_count = 0;
    for (size_t i = 0; i < fix.blocks.avail(); ++i)
    {
        auto &block = fix.blocks.get(fix.blocks.fixup_index(fix.blocks.tail_index() + i));
        if (!fix.is_block_valid(block))
        {
            invalid_count++;
            if (invalid_count <= 5)  // Only print first 5
            {
                MESSAGE("Block " << i << " INVALID: fullpath=" << block.fullpath
                        << " finish_ic=" << block.block_finish_ic);
            }
        }
    }

    int error_count = fix.count_errors();
    MESSAGE("Invalid blocks: " << invalid_count << "/" << fix.blocks.avail());
    MESSAGE("Errors reported: " << error_count);

    CHECK_EQ(invalid_count, 0);
    CHECK_EQ(error_count, 0);
}

TEST_CASE("spiral_lookahead: junction velocity calculation")
{
    SpiralLookaheadFixture fix;
    fix.setup();

    // Test junction velocity between two 10-degree segments
    cnc_float_type angle_deg = 10.0;
    cnc_float_type angle_rad = angle_deg * M_PI / 180.0;

    // Two segments at 10 degree angle
    std::array<cnc_float_type, NMAX_AXES> dir1 = {1.0, 0.0};
    std::array<cnc_float_type, NMAX_AXES> dir2 = {std::cos(angle_rad), std::sin(angle_rad)};

    cnc_float_type vel_pulses = fix.velocity * fix.control_scale / fix.freq;
    cnc_float_type acc_pulses = fix.acceleration * fix.control_scale / (fix.freq * fix.freq);
    cnc_float_type jd_pulses = fix.junction_deviation * fix.control_scale;

    cnc_float_type jv = cnc::planner_block::calculate_junction_velocity(
        dir1, dir2, vel_pulses, acc_pulses, jd_pulses, 2);

    MESSAGE("=== Junction velocity for " << angle_deg << "° turn ===");
    MESSAGE("nominal_velocity = " << vel_pulses << " pulses/tick");
    MESSAGE("acceleration = " << acc_pulses << " pulses/tick²");
    MESSAGE("junction_deviation = " << jd_pulses << " pulses");
    MESSAGE("junction_velocity = " << jv << " pulses/tick");
    MESSAGE("junction_velocity / nominal = " << (jv / vel_pulses * 100) << "%");

    // Junction velocity should be positive but less than nominal for a turn
    CHECK_GT(jv, 0);
    CHECK_LE(jv, vel_pulses);

    // For small angle (10°), junction velocity should be high (>50% of nominal)
    CHECK_GT(jv, vel_pulses * 0.5);

    // Check no NaN/Inf
    CHECK(!std::isnan(jv));
    CHECK(!std::isinf(jv));
}

TEST_CASE("spiral_lookahead: very small segments")
{
    SpiralLookaheadFixture fix;
    fix.setup();

    // Very small radius = very short segments
    auto segments = fix.generate_spiral(36, 2 * M_PI, 0.001);  // 1mm radius

    MESSAGE("=== Very small segments (1mm radius circle) ===");

    for (auto &seg : segments)
    {
        MESSAGE("Segment length: " << std::sqrt(seg.first*seg.first + seg.second*seg.second) * fix.control_scale << " pulses");
    }

    for (size_t i = 0; i < segments.size(); ++i)
    {
        fix.add_block(segments[i].first, segments[i].second);
    }

    // Apply look-ahead
    cnc_float_type jd_pulses = fix.junction_deviation * fix.control_scale;
    fix.planner.recalculate_block_velocities(jd_pulses);

    int error_count = fix.count_errors();
    MESSAGE("Errors: " << error_count);

    // Small segments should still work without errors
    CHECK_EQ(error_count, 0);
}

TEST_CASE("spiral_lookahead: recalculate_timing edge cases")
{
    // Test recalculate_timing with various boundary velocities
    SpiralLookaheadFixture fix;
    fix.setup();

    cnc_float_type vel_pulses = fix.velocity * fix.control_scale / fix.freq;
    cnc_float_type acc_pulses = fix.acceleration * fix.control_scale / (fix.freq * fix.freq);

    SUBCASE("start_velocity = final_velocity = 0")
    {
        cnc_float_type task[] = {100 * fix.control_scale, 0};
        ralgo::vector_view<cnc_float_type> view{task, 2};

        cnc::planner_block block;
        block.set_state(view, 2, vel_pulses, acc_pulses, 0, 0);

        CHECK(fix.is_block_valid(block));
        CHECK_EQ(block.start_velocity, 0);
        CHECK_EQ(block.final_velocity, 0);
    }

    SUBCASE("start_velocity = final_velocity = nominal")
    {
        cnc_float_type task[] = {100 * fix.control_scale, 0};
        ralgo::vector_view<cnc_float_type> view{task, 2};

        cnc::planner_block block;
        block.set_state(view, 2, vel_pulses, acc_pulses, vel_pulses, vel_pulses);

        CHECK(fix.is_block_valid(block));
        CHECK_EQ(block.start_velocity, vel_pulses);
        CHECK_EQ(block.final_velocity, vel_pulses);
        // Should be pure cruise (no accel/decel time)
        CHECK_EQ(block.acceleration_before_ic, 0);
    }

    SUBCASE("start > final")
    {
        cnc_float_type task[] = {100 * fix.control_scale, 0};
        ralgo::vector_view<cnc_float_type> view{task, 2};

        cnc::planner_block block;
        block.set_state(view, 2, vel_pulses, acc_pulses, vel_pulses * 0.8, vel_pulses * 0.2);

        CHECK(fix.is_block_valid(block));
    }

    SUBCASE("start < final")
    {
        cnc_float_type task[] = {100 * fix.control_scale, 0};
        ralgo::vector_view<cnc_float_type> view{task, 2};

        cnc::planner_block block;
        block.set_state(view, 2, vel_pulses, acc_pulses, vel_pulses * 0.2, vel_pulses * 0.8);

        CHECK(fix.is_block_valid(block));
    }

    SUBCASE("very short path with junction velocities")
    {
        // Path so short it can't reach nominal - triangle profile
        cnc_float_type task[] = {1 * fix.control_scale, 0};  // 1 unit
        ralgo::vector_view<cnc_float_type> view{task, 2};

        cnc::planner_block block;
        block.set_state(view, 2, vel_pulses, acc_pulses, vel_pulses * 0.5, 0);

        MESSAGE("Short path block:");
        MESSAGE("  fullpath = " << block.fullpath);
        MESSAGE("  is_triangle = " << block.is_triangle());
        MESSAGE("  block_finish_ic = " << block.block_finish_ic);

        CHECK(fix.is_block_valid(block));
    }
}

TEST_CASE("spiral_lookahead: sequential lookahead recalculations")
{
    // Simulate what happens when blocks are added one by one with look-ahead
    SpiralLookaheadFixture fix;
    fix.setup();

    auto segments = fix.generate_spiral(20, M_PI, 0.1);  // Half circle
    cnc_float_type jd_pulses = fix.junction_deviation * fix.control_scale;

    MESSAGE("=== Sequential add + recalc ===");

    for (size_t i = 0; i < segments.size(); ++i)
    {
        fix.add_block(segments[i].first, segments[i].second);

        // Recalculate after each block (like interpreter does)
        fix.planner.recalculate_block_velocities(jd_pulses);

        // Verify last block is still valid
        int last_idx = fix.blocks.fixup_index(fix.blocks.head_index() - 1);
        auto &last_block = fix.blocks.get(last_idx);

        if (!fix.is_block_valid(last_block))
        {
            MESSAGE("Block " << i << " became INVALID after recalc:");
            MESSAGE("  fullpath=" << last_block.fullpath);
            MESSAGE("  block_finish_ic=" << last_block.block_finish_ic);
            MESSAGE("  start_velocity=" << last_block.start_velocity);
            MESSAGE("  final_velocity=" << last_block.final_velocity);
        }

        CHECK(fix.is_block_valid(last_block));
    }

    int error_count = fix.count_errors();
    CHECK_EQ(error_count, 0);
}

TEST_CASE("spiral_lookahead: concurrent execution and addition")
{
    // Simulate planner executing blocks while new blocks are added
    // This is closer to what happens in real helix-cnc
    SpiralLookaheadFixture fix;
    fix.setup();

    auto segments = fix.generate_spiral(50, 3 * M_PI, 0.1);  // 1.5 rotations
    cnc_float_type jd_pulses = fix.junction_deviation * fix.control_scale;

    MESSAGE("=== Concurrent execution + addition ===");

    size_t seg_idx = 0;
    int total_ticks = 0;
    int blocks_executed = 0;

    // Add first few blocks to start
    for (int i = 0; i < 5 && seg_idx < segments.size(); ++i, ++seg_idx)
    {
        fix.add_block(segments[seg_idx].first, segments[seg_idx].second);
    }
    fix.planner.recalculate_block_velocities(jd_pulses);

    // Simulate execution loop
    while (fix.blocks.avail() > 0 || seg_idx < segments.size())
    {
        // Execute some ticks
        for (int tick = 0; tick < 1000; ++tick)
        {
            auto [fin, iters] = fix.planner.iteration();
            total_ticks += iters;
            if (fin)
            {
                blocks_executed++;
                break;
            }
        }

        // Add new block if available and queue has room
        if (seg_idx < segments.size() && fix.blocks.room() > 2)
        {
            fix.add_block(segments[seg_idx].first, segments[seg_idx].second);
            seg_idx++;

            // Recalculate (this is where corruption might happen)
            fix.planner.recalculate_block_velocities(jd_pulses);
        }

        // Check all pending blocks are valid
        for (size_t i = 0; i < fix.blocks.avail(); ++i)
        {
            int idx = fix.blocks.fixup_index(fix.blocks.tail_index() + i);
            auto &block = fix.blocks.get(idx);
            if (!fix.is_block_valid(block))
            {
                MESSAGE("Block " << i << " INVALID during execution:");
                MESSAGE("  fullpath=" << block.fullpath);
                MESSAGE("  block_finish_ic=" << block.block_finish_ic);
                FAIL("Block became invalid during concurrent operation");
            }
        }

        // Safety limit
        if (total_ticks > 100000000)
        {
            MESSAGE("Timeout - too many ticks");
            break;
        }
    }

    MESSAGE("Total ticks: " << total_ticks);
    MESSAGE("Blocks executed: " << blocks_executed);
    MESSAGE("Segments processed: " << seg_idx);

    int error_count = fix.count_errors();
    MESSAGE("Errors: " << error_count);

    CHECK_EQ(error_count, 0);
    CHECK_EQ(seg_idx, segments.size());
}

TEST_CASE("spiral_lookahead: active block with lookahead")
{
    // Test when active_block is set and lookahead recalculates pending blocks
    SpiralLookaheadFixture fix;
    fix.setup();

    auto segments = fix.generate_spiral(10, M_PI / 2, 0.1);  // Quarter circle
    cnc_float_type jd_pulses = fix.junction_deviation * fix.control_scale;

    MESSAGE("=== Active block + lookahead ===");

    // Add all blocks
    for (auto &seg : segments)
    {
        fix.add_block(seg.first, seg.second);
    }

    // Start execution - this sets active_block
    fix.planner.serve();

    MESSAGE("active_block set: " << (fix.planner.active_block != nullptr));

    // Now add more blocks while active_block is set
    auto more_segments = fix.generate_spiral(10, M_PI / 2, 0.1);
    for (auto &seg : more_segments)
    {
        fix.add_block(seg.first, seg.second);
        fix.planner.recalculate_block_velocities(jd_pulses);
    }

    // Execute until done
    int max_iters = 10000000;
    while (fix.planner.is_not_halt() && max_iters-- > 0)
    {
        fix.planner.serve();
    }

    int error_count = fix.count_errors();
    MESSAGE("Errors: " << error_count);
    CHECK_EQ(error_count, 0);
}

TEST_CASE("spiral_lookahead: backward/forward pass edge cases")
{
    // Test backward/forward pass with blocks that have extreme velocity ratios
    SpiralLookaheadFixture fix;
    fix.setup();

    cnc_float_type vel_pulses = fix.velocity * fix.control_scale / fix.freq;
    cnc_float_type acc_pulses = fix.acceleration * fix.control_scale / (fix.freq * fix.freq);
    cnc_float_type jd_pulses = fix.junction_deviation * fix.control_scale;

    MESSAGE("=== Backward/forward pass edge cases ===");

    SUBCASE("Very short followed by very long")
    {
        // Short block can't decelerate from high velocity
        cnc_float_type short_task[] = {10, 0};  // 10 pulses
        cnc_float_type long_task[] = {100000, 0};  // 100000 pulses

        ralgo::vector_view<cnc_float_type> short_view{short_task, 2};
        ralgo::vector_view<cnc_float_type> long_view{long_task, 2};

        auto &block1 = fix.blocks.head_place();
        block1.set_state(short_view, 2, vel_pulses, acc_pulses);
        fix.blocks.move_head_one();

        auto &block2 = fix.blocks.head_place();
        block2.set_state(long_view, 2, vel_pulses, acc_pulses);
        fix.blocks.move_head_one();

        fix.planner.recalculate_block_velocities(jd_pulses);

        CHECK(fix.is_block_valid(block1));
        CHECK(fix.is_block_valid(block2));

        MESSAGE("Short block final_velocity: " << block1.final_velocity);
        MESSAGE("Long block start_velocity: " << block2.start_velocity);

        // Velocities should match at junction
        CHECK(std::abs(block1.final_velocity - block2.start_velocity) < 1e-10);
    }

    SUBCASE("90 degree turn requires slowdown")
    {
        cnc_float_type task1[] = {10000, 0};
        cnc_float_type task2[] = {0, 10000};

        ralgo::vector_view<cnc_float_type> view1{task1, 2};
        ralgo::vector_view<cnc_float_type> view2{task2, 2};

        auto &block1 = fix.blocks.head_place();
        block1.set_state(view1, 2, vel_pulses, acc_pulses);
        fix.blocks.move_head_one();

        auto &block2 = fix.blocks.head_place();
        block2.set_state(view2, 2, vel_pulses, acc_pulses);
        fix.blocks.move_head_one();

        fix.planner.recalculate_block_velocities(jd_pulses);

        CHECK(fix.is_block_valid(block1));
        CHECK(fix.is_block_valid(block2));

        MESSAGE("Block 1 final_velocity: " << block1.final_velocity);
        MESSAGE("Block 2 start_velocity: " << block2.start_velocity);

        // Junction velocity should be less than nominal for 90 degree turn
        CHECK(block1.final_velocity < vel_pulses);
    }

    int error_count = fix.count_errors();
    CHECK_EQ(error_count, 0);
}
