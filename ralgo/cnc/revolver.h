#ifndef RALGO_REVOLVER_H
#define RALGO_REVOLVER_H

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <igris/container/ring.h>
#include <igris/event/delegate.h>
#include <igris/sync/syslock.h>
#include <ralgo/cnc/defs.h>
#include <ralgo/robo/stepper.h>

namespace cnc
{
    /**
     * Task for DDA execution.
     * Contains accelerations in fixed-point format and duration in ticks.
     *
     * Units:
     * - accelerations_fixed: steps/tick² * FIXED_POINT_MUL
     * - duration_ticks: number of timer ticks to execute
     */
    class revolver_task
    {
    public:
        std::array<fixed_t, NMAX_AXES> accelerations_fixed = {};
        int32_t duration_ticks = 0;

    public:
        revolver_task() = default;

        /**
         * Create task from accelerations in steps/tick² and duration.
         * @param accel Accelerations in steps/tick² (will be converted to fixed-point)
         * @param ticks Duration in timer ticks
         */
        revolver_task(const std::array<cnc_float_type, NMAX_AXES> &accel,
                      int32_t ticks)
            : duration_ticks(ticks)
        {
            for (size_t i = 0; i < NMAX_AXES; ++i)
            {
                accelerations_fixed[i] =
                    static_cast<fixed_t>(accel[i] * FIXED_POINT_MUL);
            }
        }
    };

    /**
     * DDA (Digital Differential Analyzer) step generator.
     *
     * Executes motion by integrating accelerations to velocities,
     * and velocities to positions. Generates step pulses when
     * position crosses integer boundaries.
     *
     * All internal calculations are in fixed-point for deterministic
     * timing on embedded systems.
     *
     * Units:
     * - velocities_fixed: steps/tick * FIXED_POINT_MUL
     * - positions_fixed: steps * FIXED_POINT_MUL (fractional position)
     */
    class revolver
    {
    public:
        int steppers_total = 0;
        int32_t ticks_remaining = 0;

    public:
        igris::ring<cnc::revolver_task> task_queue;
        robo::stepper **steppers = nullptr;
        cnc::revolver_task *current_task = nullptr;

        // Compatibility aliases for old names
        igris::ring<cnc::revolver_task> &revolver_task_ring = task_queue;
        cnc::revolver_task *&current_revolver_task = current_task;

        // DDA state (fixed-point)
        std::array<fixed_t, NMAX_AXES> velocities_fixed = {};
        std::array<fixed_t, NMAX_AXES> positions_fixed = {};

    public:
        revolver();

        // Stepper management
        void set_steppers(robo::stepper **steppers_table, int count);
        robo::stepper **get_steppers();

        // Queue management
        bool is_empty();
        int queue_room();
        int queue_size();

        // Main DDA execution (called from timer interrupt)
        void serve();

        // State queries
        void get_current_steps(steps_t *out_steps);
        std::vector<steps_t> get_current_steps();
        std::vector<steps_t> get_current_steps_no_lock();
        std::array<cnc_float_type, NMAX_AXES> get_current_velocities() const;

        // Reset/clear
        void clear();
        void clear_keep_velocity();
        void reset();

        // Compatibility aliases (will be removed after full refactoring)
        void current_steps(steps_t *out) { get_current_steps(out); }
        std::vector<steps_t> current_steps() { return get_current_steps(); }
        std::vector<steps_t> current_steps_no_lock()
        {
            return get_current_steps_no_lock();
        }
        std::array<cnc_float_type, NMAX_AXES> current_velocities() const
        {
            return get_current_velocities();
        }
        void cleanup() { clear(); }
        int room() { return queue_room(); }
        void clear_no_velocity_drop() { clear_keep_velocity(); }

    private:
        void advance_to_next_task();
    };
}

#endif
