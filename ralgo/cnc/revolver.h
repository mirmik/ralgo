#ifndef RALGO_REVOLVER_H
#define RALGO_REVOLVER_H

#include <stdint.h>

#include <cstdlib>
#include <igris/container/ring.h>
#include <igris/sync/syslock.h>

#include <igris/event/delegate.h>
#include <ralgo/cnc/planblock.h>
#include <ralgo/cnc/planner.h>
#include <ralgo/cnc/shift.h>
#include <ralgo/log.h>
#include <ralgo/robo/stepper.h>

namespace cnc
{

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
        bool info_mode = false;
        int steppers_total = 0;

    private:
        volatile bool all_blocks_resolved = true;

        robo::stepper **steppers = nullptr;

    public:
        igris::ring<cnc::control_shift> *shifts_ring = {};
        igris::ring<cnc::planner_block> *blocks = {};
        cnc::planner *planner = nullptr;
        igris::delegate<void> final_shift_pushed = {};

    public:
        revolver(igris::ring<cnc::control_shift> *ring,
                 igris::ring<cnc::planner_block> *blocks,
                 cnc::planner *planner)
            : shifts_ring(ring), blocks(blocks), planner(planner)
        {
        }

        robo::stepper **get_steppers()
        {
            return steppers;
        }

        void enable_simulator_mode()
        {
            system_lock();
            for (int i = 0; i < steppers_total; ++i)
            {
                steppers[i]->simulator_mode(true);
            }
            system_unlock();
        }

        void set_steppers(robo::stepper **steppers_table, int size)
        {
            steppers = steppers_table;
            steppers_total = size;
        }

        int queue_size()
        {
            int size;

            system_lock();
            size = shifts_ring->avail();
            system_unlock();

            return size;
        }

        void current_velocity(double *velocity)
        {
            system_lock();
            if (shifts_ring->empty() && blocks->avail() == 0)
            {
                for (int i = 0; i < steppers_total; ++i)
                    velocity[i] = 0;
            }
            else
                for (int i = 0; i < steppers_total; ++i)
                    velocity[i] = shifts_ring->tail().speed[i];
            system_unlock();
        }

        std::vector<double> current_velocity_no_lock(nos::ostream &)
        {
            std::vector<double> vec(steppers_total);
            if (shifts_ring->empty() && blocks->avail() == 0)
            {
                for (int i = 0; i < steppers_total; ++i)
                    vec[i] = 0;
            }
            else
            {
                for (int i = 0; i < steppers_total; ++i)
                    vec[i] = shifts_ring->tail().speed[i];
            }
            return vec;
        }

        void current_steps(int64_t *steps)
        {
            system_lock();
            for (int i = 0; i < steppers_total; ++i)
                steps[i] = steppers[i]->steps_count();
            system_unlock();
        }

        std::vector<int64_t> current_steps()
        {
            std::vector<int64_t> vec(steppers_total);
            system_lock();
            for (int i = 0; i < steppers_total; ++i)
                vec[i] = steppers[i]->steps_count();
            system_unlock();
            return vec;
        }

        std::vector<int64_t> current_steps_no_lock()
        {
            std::vector<int64_t> vec(steppers_total);
            for (int i = 0; i < steppers_total; ++i)
                vec[i] = steppers[i]->steps_count();
            return vec;
        }

        int room()
        {
            // Операция взятия оставшегося места над кольцевым буфером
            // требует сравнения head и tail, изменяемых в разных потоках,
            // поэтому не является атомарной.
            system_lock();
            int ret = shifts_ring->room();
            system_unlock();

            return ret;
        }

        void push(uint16_t step, uint16_t dir)
        {
            // Добавление данных в очередь не требует блокировки,
            // потому что ring_head lockfree на добавление и чтение,
            // если unsigned int атомарен.

            shifts_ring->emplace(step, dir);
        }

        void serve()
        {
            // В общем случае ring_empty не атомарен. Однако, здесь должен
            // быть контекст приоритетного прерывания.
            if (shifts_ring->empty())
            {
                if (all_blocks_resolved == false && blocks->empty() &&
                    planner->active_block == nullptr &&
                    planner->active == blocks->tail_index())
                {
                    all_blocks_resolved = true;
                    final_shift_pushed();
                }
                return;
            }
            else
            {
                all_blocks_resolved = false;
            }

            // int idx = shifts_ring->tail_index();
            auto &shift = shifts_ring->tail();

            for (int i = 0; i < steppers_total; ++i)
            {
                revolver_t mask = 1 << i;
                bool step = shift.step & mask;

                if (!step)
                    continue;

                bool dir = shift.direction & mask;

                if (dir)
                    steppers[i]->inc();
                else
                    steppers[i]->dec();
            }

            shifts_ring->move_tail_one();
            return;
        }

        void clear()
        {
            shifts_ring->clear();
            all_blocks_resolved = true;
        }
    };
}

#endif
