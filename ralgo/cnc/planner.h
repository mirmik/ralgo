#ifndef RALGO_PLANNER_RING_H
#define RALGO_PLANNER_RING_H

#include <stdint.h>

#include <ralgo/cnc/planblock.h>
#include <ralgo/cnc/shift.h>
#include <ralgo/linalg/vecops.h>

#include <igris/container/array_view.h>
#include <igris/container/ring.h>
#include <igris/container/static_vector.h>
#include <igris/datastruct/dlist.h>
#include <igris/event/delegate.h>
#include <igris/sync/syslock.h>

#include <nos/fprint.h>
#include <ralgo/log.h>

#define NMAX_AXES 10

namespace cnc
{
    /**
     * Structure of blocks ring:
     *
     * accessors:   blocks.tail()
     *                  |
     *          :       |               acceleration
     * signation:       |                   or        (place for new block)
     *          :       |  deceleration   cruis               |
     *                  |  |    |     |     |                 |
     *                  v  v    v     v     v                 v
     *      ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----
     *     |     |     |     |     |     |     |     |     |     |     |     |
     *     |     |     |  .  |  .  |  .  |  .  |  .  |  .  |     |     |     |
     *     |     |     |     |     |     |     |     |     |     |     |     |
     *      ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----
     *                    ^                 ^                 ^
     *                    |                 |                 |
     * counters:       (tail)--->       (active)--->       (head)--->
     * */
    class planner
    {
    private:
        int total_axes = 0;

    public:
        bool info_mode = true;
        int64_t iteration_counter = 0;
        igris::static_vector<double, NMAX_AXES> gears = {};
        igris::static_vector<double, NMAX_AXES> gears_high_trigger = {};
        std::array<double, NMAX_AXES> accelerations = {};
        std::array<double, NMAX_AXES> velocities = {};
        std::array<double, NMAX_AXES> dda_counters = {};
        int active = 0; // index of active block
        planner_block *active_block = nullptr;
        igris::ring<cnc::planner_block> *blocks = {};
        igris::ring<cnc::control_shift> *shifts = {};
        bool need_to_reevaluate = false;
        bool in_operation = false;
        uint8_t state = 0;
        // int count_of_reevaluation = 0;
        igris::delegate<void> _start_operation_handle = {};
        int waited = 0;

        bool dda_counter_overflow_error_detected = false;

        igris::delegate<void> final_shift_pushed = {};

    public:
        planner(const planner &) = delete;
        planner(planner &&) = delete;
        planner &operator=(const planner &) = delete;
        planner &operator=(planner &&) = delete;

        void cleanup();
        bool is_dda_overflow_detected();
        void set_start_operation_handle(igris::delegate<void> dlg);
        void force_skip_all_blocks();
        void set_current_velocity(const std::vector<double> &vel);
        void set_current_velocity(const std::array<double, NMAX_AXES> &vel);
        void update_triggers();
        void set_dim(int axes);
        void reset_iteration_counter();
        planner(igris::ring<cnc::planner_block> *blocks,
                igris::ring<cnc::control_shift> *shifts);
        bool has_postactive_blocks();
        int count_of_postactive();
        void change_active_block();
        const std::array<double, NMAX_AXES> &current_velocity();
        int serve();
        void evaluate_accelerations();
        void iteration_planning_phase();
        int iteration();
        void set_axes_count(int total);
        void set_gears(const igris::array_view<double> &);
        igris::array_view<double> get_gears();
        size_t get_total_axes();
        void set_gear(int index, double val);
        void clear();
        void clear_queue();
        int block_index(planner_block *it);
        void fixup_postactive_blocks();
    };
}

#endif
