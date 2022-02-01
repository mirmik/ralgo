#ifndef RALGO_PLANNER_RING_H
#define RALGO_PLANNER_RING_H

#include <stdint.h>

#include <ralgo/linalg/vecops.h>
#include <ralgo/cnc/planblock.h>
#include <ralgo/cnc/shift.h>

#include <igris/container/array_view.h>
#include <igris/container/static_vector.h>
#include <igris/container/ring.h>
#include <igris/datastruct/dlist.h>
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
        bool info_mode = false;
        bool first_iteration_label = false;

        int64_t iteration_counter = 0;

        double delta = 1;
        double delta_sqr_div_2 = 0.5;

        igris::static_vector<double, NMAX_AXES> gears;
        igris::static_vector<double, NMAX_AXES> gears_low_trigger;
        igris::static_vector<double, NMAX_AXES> gears_high_trigger;
        double accelerations[NMAX_AXES];
        double velocities[NMAX_AXES];
        double dda_counters[NMAX_AXES];

        int active = 0; // index of active block
        planner_block *active_block = nullptr;

        igris::ring<cnc::planner_block> *blocks;
        igris::ring<cnc::control_shift> *shifts;

        bool need_to_reevaluate = false;
        uint8_t state = 0;
        int count_of_reevaluation = 0;

    public:
        void update_triggers() 
        {
            for (unsigned int i = 0; i < gears.size(); ++i) 
            {
                gears_low_trigger[i] = gears[i] * 0.1;
                gears_high_trigger[i] = gears[i] * 0.9;
            }
        }

        void set_dim(int axes) { total_axes = axes; }

        /*void set_revolver_delta(double delta)
        {
            this->delta = delta;
            this->delta_sqr_div_2 = delta * delta / 2;
        }*/

        void reset_iteration_counter() { iteration_counter = 0; }

        planner(igris::ring<cnc::planner_block> *blocks,
                igris::ring<cnc::control_shift> *shifts)
            : blocks(blocks), shifts(shifts)
        {
            memset(accelerations, 0, sizeof(accelerations));
            memset(velocities, 0, sizeof(velocities));
            memset(dda_counters, 0, sizeof(dda_counters));
        }

        int block_index(planner_block *it) { return blocks->index_of(it); }

        //void synchronize_finished_block(planner_block &block)
        //{
        //    for (int i = 0; i < total_axes; ++i)
        //    {
        //        synced_steps[i] += block.steps[i];
        //    }
        //}

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
            static int waited = 0;

            if (info_mode)
            {
                ralgo::info("planner: change_active_block");
            }

            if (active_block && has_postactive_blocks() == 0 &&
                iteration_counter == active_block->active_finish_ic)
            {
                // TODO: normalize time
            }

            if (active_block)
                active = blocks->fixup_index(active + 1);

            system_lock();
            int head = blocks->head_index();
            system_unlock();

            if (active == head)
            {
                active_block = nullptr;
                return;
            }

            active_block = &blocks->get(active);
            ralgo::info("GET_NEW_BLOCK: {}", nos::format("{}", *active_block).c_str());

            assert(active_block->blockno == waited);
            waited++;

            active_block->shift_timestampes(iteration_counter);
        }

        int serve()
        {
            int final;

            if (first_iteration_label == false)
            {
                first_iteration_label = true;
                ralgo::info("planner: first start. success");
            }

            system_lock();
            int room = shifts->room();
            // bool empty = blocks->empty();
            system_unlock();

            /*if (active_block == nullptr && !has_postactive_blocks())
            {
                if (empty)
                    return 1;

                else
                    start_with_first_block();
            }*/

            while (room--)
            {
                // Планируем поведение револьвера на несколько циклов вперёд
                // попутно инкрементируя модельное время.
                final = iteration();

                if (final)
                    return final;
            }

            return 0;
        }

        void evaluate_accelerations()
        {
            if (active_block)
                active_block->assign_accelerations(accelerations, total_axes,
                                                   iteration_counter);
            else
            {
                for (int i = 0; i < total_axes; ++i)
                    accelerations[i] = 0;
            }

            for (int i = blocks->tail_index(); i != active;
                 i = blocks->fixup_index(i + 1))
                blocks->get(i).append_accelerations(accelerations, total_axes,
                                                    iteration_counter);
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
                }
                else if (dda_counters[i] < -gears_high_trigger[i])
                {
                    dda_counters[i] += gears[i];
                    dir |= mask;
                }
                velocities[i] += accelerations[i]; // * delta;
            }
            shifts->emplace(dir, step, velocities, total_axes);

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
                count_of_reevaluation++;
            }

            iteration_planning_phase();
            return 0;
        }

        void set_axes_count(int total) 
        { 
            total_axes = total;
            gears.resize(total);
            gears_low_trigger.resize(total);
            gears_high_trigger.resize(total);
            ralgo::vecops::fill(gears, 100000);
            update_triggers(); 
        }

        void set_gears(igris::array_view<double> arr) 
        {
            ralgo::vecops::copy(arr, gears);
            update_triggers();
        }

        igris::array_view<double> get_gears() 
        {
            return { gears.data(), gears.size() };
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
            change_active_block();
        }
    };
}

#endif
