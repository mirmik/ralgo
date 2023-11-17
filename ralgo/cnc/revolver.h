#ifndef RALGO_REVOLVER_H
#define RALGO_REVOLVER_H

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <igris/container/ring.h>
#include <igris/event/delegate.h>
#include <igris/sync/syslock.h>
#include <ralgo/cnc/defs.h>
#include <ralgo/cnc/planblock.h>
#include <ralgo/cnc/shift.h>
#include <ralgo/log.h>
#include <ralgo/robo/stepper.h>

namespace cnc
{
    class revolver_task
    {
    public:
        std::array<cnc_float_type, NMAX_AXES> accelerations;
        int64_t step_start;
        int64_t step_end;

    public:
        revolver_task() = default;
        revolver_task(const std::array<cnc_float_type, NMAX_AXES> &accel,
                      int64_t step_start,
                      int64_t step_end)
            : accelerations(accel), step_start(step_start), step_end(step_end)
        {
        }
    };

    /**
     * Structure of shifts ring:
     *
     *              blocks.tail()                     (place for new shift)
     *                    |                                   |
     *                    |                                   |
     *                    v                                   v
     *      ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----
     *     |     |     |     |     |     |     |     |     |     |     |     |
     *     |     |     |  .  |  .  |  .  |  .  |  .  |  .  |     |     |     |
     *     |     |     |     |     |     |     |     |     |     |     |     |
     *      ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----
     *                    ^                                   ^
     *                    |                                   |
     * counters:       (tail)-->                           (head)-->
     * */
    class revolver
    {
    public:
        int steppers_total = 0;
        int64_t iteration_counter = 0;

    public:
        robo::stepper **steppers = nullptr;
        // igris::ring<cnc::control_shift> *shifts_ring = {};
        igris::ring<cnc::revolver_task> revolver_task_ring = {};
        cnc::revolver_task *current_revolver_task = {};
        control_shift zero_step = {};

        std::array<cnc_float_type, NMAX_AXES> velocities = {};
        std::array<cnc_float_type, NMAX_AXES> dda_counters = {};

        igris::static_vector<cnc_float_type, NMAX_AXES> gears = {};
        igris::static_vector<cnc_float_type, NMAX_AXES> gears_high_trigger = {};

    public:
        void change_task();

        const std::array<cnc_float_type, NMAX_AXES> &current_velocities() const
        {
            return velocities;
        }
        revolver();
        void cleanup();
        bool is_empty();
        robo::stepper **get_steppers();
        void set_steppers(robo::stepper **steppers_table, int size);
        int queue_size();
        void current_steps(int64_t *steps);
        std::vector<int64_t> current_steps();
        std::vector<int64_t> current_steps_no_lock();
        int room();
        void push(uint16_t step, uint16_t dir);
        void serve();
        void clear();
        void clear_no_velocity_drop();
    };
}

#endif
