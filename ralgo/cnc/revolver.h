#ifndef RALGO_REVOLVER_H
#define RALGO_REVOLVER_H

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <igris/container/ring.h>
#include <igris/event/delegate.h>
#include <igris/sync/syslock.h>
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
        int steppers_total = 0;

    private:
        robo::stepper **steppers = nullptr;
        igris::ring<cnc::control_shift> *shifts_ring = {};

    public:
        revolver(igris::ring<cnc::control_shift> *ring);
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
    };
}

#endif
