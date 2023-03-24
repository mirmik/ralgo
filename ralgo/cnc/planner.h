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
        double accelerations[NMAX_AXES];
        double velocities[NMAX_AXES];
        double dda_counters[NMAX_AXES];
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

        void cleanup()
        {
            blocks->clear();
            shifts->clear();
            shifts->reset();
            blocks->reset();
            active_block = nullptr;
            active = blocks->head_index();
            state = 0;
            memset(accelerations, 0, sizeof(accelerations));
            memset(velocities, 0, sizeof(velocities));
            memset(dda_counters, 0, sizeof(dda_counters));
            need_to_reevaluate = false;
            state = 0;
            waited = 0;
        }

        bool is_dda_overflow_detected()
        {
            return dda_counter_overflow_error_detected;
        }

        void set_start_operation_handle(igris::delegate<void> dlg)
        {
            _start_operation_handle = dlg;
        }

        void force_skip_all_blocks()
        {
            blocks->clear();
            active_block = nullptr;
            active = blocks->head_index();
            state = 0;
            memset(accelerations, 0, sizeof(accelerations));
            // memset(velocities, 0, sizeof(velocities));
            // memset(dda_counters, 0, sizeof(dda_counters));
        }

        void set_current_velocity(std::vector<double> vel)
        {
            for (int i = 0; i < total_axes; ++i)
            {
                velocities[i] = vel[i];
            }
        }

        void update_triggers()
        {
            for (unsigned int i = 0; i < gears.size(); ++i)
            {
                gears_high_trigger[i] = gears[i] * 0.9;
            }
        }

        void set_dim(int axes)
        {
            total_axes = axes;
        }

        void reset_iteration_counter()
        {
            iteration_counter = 0;
        }

        planner(igris::ring<cnc::planner_block> *blocks,
                igris::ring<cnc::control_shift> *shifts)
            : blocks(blocks), shifts(shifts)
        {
            memset(accelerations, 0, sizeof(accelerations));
            memset(velocities, 0, sizeof(velocities));
            memset(dda_counters, 0, sizeof(dda_counters));
        }

        int block_index(planner_block *it)
        {
            return blocks->index_of(it);
        }

        void fixup_postactive_blocks()
        {
            while (blocks->tail_index() != active)
            {
                if (!blocks->tail().is_active_or_postactive(iteration_counter))
                    blocks->pop();

                else
                    break;
            }
        }

        bool has_postactive_blocks()
        {
            system_lock();
            auto ret = active != blocks->tail_index();
            system_unlock();
            return ret;
        }

        int count_of_postactive()
        {
            system_lock();
            auto ret = blocks->distance(active, blocks->tail_index());
            system_unlock();
            return ret;
        }

        void change_active_block()
        {
            if (info_mode)
            {
                ralgo::info("planner: change_active_block");
            }

            system_lock();
            if (active_block && has_postactive_blocks() == 0 &&
                iteration_counter == active_block->active_finish_ic)
            {
                // TODO: normalize time
            }

            if (active_block)
                active = blocks->fixup_index(active + 1);

            int head = blocks->head_index();
            system_unlock();

            if (active_block == nullptr && active != head)
            {
                _start_operation_handle();
            }

            if (active == head)
            {
                active_block = nullptr;
                return;
            }

            active_block = &blocks->get(active);

            assert(active_block->blockno == waited);
            waited++;

            active_block->shift_timestampes(iteration_counter);
        }

        int serve()
        {
            int final;

            system_lock();
            bool shifts_empty = shifts->empty();
            bool blocks_empty = blocks->empty() && active_block == nullptr;
            if (shifts_empty && blocks_empty && in_operation)
            {
                in_operation = false;
                final_shift_pushed();
                system_unlock();
                return 0;
            }
            int room = shifts->room();
            system_unlock();

            while (room--)
            {
                // Планируем поведение револьвера на несколько циклов вперёд
                // попутно инкрементируя модельное время.
                final = iteration();
                in_operation = true;

                if (final)
                    return final;
            }

            return 0;
        }

        void evaluate_accelerations()
        {
            if (active_block)
                active_block->assign_accelerations(
                    accelerations, total_axes, iteration_counter);
            else
            {
                for (int i = 0; i < total_axes; ++i)
                    accelerations[i] = 0;
            }

            for (int i = blocks->tail_index(); i != active;
                 i = blocks->fixup_index(i + 1))
                blocks->get(i).append_accelerations(
                    accelerations, total_axes, iteration_counter);
        }

        /// В этой фазе расчитывается программе револьвера
        /// на основе интегрирования ускорений и скоростей.
        void iteration_planning_phase()
        {
            revolver_t mask, step = 0, dir = 0;

            for (int i = 0; i < total_axes; ++i)
            {
                mask = (1 << i);

                dda_counters[i] += velocities[i] +         //* delta +
                                   accelerations[i] * 0.5; //* delta_sqr_div_2;

                if (dda_counters[i] > gears_high_trigger[i])
                {
                    dda_counters[i] -= gears[i];
                    dir |= mask;
                    step |= mask;

                    if (dda_counters[i] >= gears[i])
                    {
                        dda_counter_overflow_error_detected = true;
                    }
                }
                else if (dda_counters[i] < -gears_high_trigger[i])
                {
                    dda_counters[i] += gears[i];
                    dir |= mask;

                    if (dda_counters[i] <= gears[i])
                    {
                        dda_counter_overflow_error_detected = true;
                    }
                }
                velocities[i] += accelerations[i]; // * delta;
            }
            system_lock();
            shifts->emplace(dir, step, velocities, total_axes);
            system_unlock();

            iteration_counter++;
        }

        int iteration()
        {
            if (active_block)
            {
                if (state == 0)
                {
                    if (!active_block->is_accel(iteration_counter))
                    {
                        need_to_reevaluate = true;
                        state = 1;
                    }
                }

                else
                {
                    if (!active_block->is_active(iteration_counter))
                    {
                        change_active_block();
                        need_to_reevaluate = true;
                        state = 0;
                    }
                }
            }

            for (int i = blocks->tail_index(); i != active;
                 i = blocks->fixup_index(i + 1))
            {
                if (!blocks->get(i).is_active_or_postactive(iteration_counter))
                {
                    need_to_reevaluate = true;
                }
            }

            if (active_block == nullptr && !has_postactive_blocks())
            {
                bool empty = blocks->empty();
                if (empty)
                    return 1;

                change_active_block();

                if (active_block == nullptr)
                    return 1;

                need_to_reevaluate = true;
            }

            if (need_to_reevaluate)
            {
                fixup_postactive_blocks();
                evaluate_accelerations();
                need_to_reevaluate = false;
                // count_of_reevaluation++;
            }

            if (active_block == nullptr && !has_postactive_blocks())
                return 1;

            iteration_planning_phase();
            return 0;
        }

        void set_axes_count(int total)
        {
            total_axes = total;
            gears.resize(total);
            gears_high_trigger.resize(total);
            ralgo::vecops::fill(gears, 1000);
            update_triggers();
        }

        void set_gears(igris::array_view<double> arr)
        {
            ralgo::vecops::copy(arr, gears);
            update_triggers();
        }

        igris::array_view<double> get_gears()
        {
            return {gears.data(), gears.size()};
        }

        size_t get_total_axes()
        {
            return total_axes;
        }

        void set_gear(int index, double val)
        {
            system_lock();
            gears[index] = val;
            update_triggers();
            system_unlock();
        }

        void clear()
        {
            blocks->clear();
            ralgo::vecops::fill(dda_counters, 0);
            ralgo::vecops::fill(accelerations, 0);
            ralgo::vecops::fill(velocities, 0);
            active_block = nullptr;
            active = blocks->head_index();
            need_to_reevaluate = true;
            state = 0;
            change_active_block();
        }

        void clear_queue()
        {
            blocks->set_last_index(active);
        }
    };
}

#endif
