#ifndef RALGO_PLANNER_RING_H
#define RALGO_PLANNER_RING_H

#include <stdint.h>

#include <igris/container/array_view.h>
#include <igris/container/ring.h>
#include <igris/container/static_vector.h>
#include <igris/datastruct/dlist.h>
#include <igris/event/delegate.h>
#include <igris/sync/syslock.h>
#include <nos/fprint.h>
#include <ralgo/cnc/planblock.h>
#include <ralgo/cnc/revolver.h>
#include <ralgo/cnc/shift.h>
#include <ralgo/cnc/util.h>
#include <ralgo/linalg/vecops.h>
#include <ralgo/log.h>

#pragma GCC optimize("Ofast")
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
        int _total_axes = 0;

    public:
        bool pause = false;
        bool info_mode = true;
        int64_t iteration_counter = 0;
        std::array<cnc_float_type, NMAX_AXES> accelerations = {};
        int active = 0; // index of active block
        planner_block *active_block = nullptr;
        igris::ring<cnc::planner_block> *blocks = {};
        cnc::revolver *revolver = nullptr;
        bool need_to_reevaluate = false;
        bool in_operation = false;
        uint8_t state = 0;
        igris::delegate<void> _start_operation_handle = {};
        int waited = 0;
        bool dda_counter_overflow_error_detected = false;
        igris::delegate<void> final_shift_pushed = {};
        bool _frequency_protection = true;

    public:
        planner(const planner &) = delete;
        planner(planner &&) = delete;
        planner &operator=(const planner &) = delete;
        planner &operator=(planner &&) = delete;

        void disable_frequency_protection();
        void cleanup();
        bool is_dda_overflow_detected();
        void set_start_operation_handle(igris::delegate<void> dlg);
        void force_skip_all_blocks();
        void update_triggers();
        void set_dim(int axes);
        void reset_iteration_counter();
        planner(igris::ring<cnc::planner_block> *blocks,
                cnc::revolver *revolver);
        bool has_postactive_blocks();
        int count_of_postactive();
        void change_active_block();
        int serve(bool prevent_small_rooms = true);
        void evaluate_accelerations();
        void iteration_planning_phase(size_t iter);
        std::pair<int, size_t> iteration();
        void set_axes_count(int total);
        size_t get_total_axes();
        size_t total_axes();
        void clear();
        int block_index(planner_block *it);
        void fixup_postactive_blocks();

        void alarm_stop();
        void clear_for_stop();

        void set_pause_mode(bool en);

        bool is_not_halt();
        bool is_halt();
        void reevaluate_accelerations();
    };
}
#pragma GCC reset_options

#endif
