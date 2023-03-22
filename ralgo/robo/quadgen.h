#ifndef RALGO_ROBO_QUADGEN_H
#define RALGO_ROBO_QUADGEN_H

#include <ralgo/robo/stepper.h>

namespace robo
{
    class quadgen : public robo::stepper
    {
        int8_t state = 0;

    public:
        void set_declared_state(uint8_t declared_state)
        {
            state = declared_state;
        }

        void inc() override
        {
            ++counter;
            auto newstate = state == 3 ? 0 : state + 1;
            set_state(newstate);
        }

        void dec() override
        {
            --counter;
            auto newstate = state == 0 ? 3 : state - 1;
            set_state(newstate);
        }

        virtual void apply_state() = 0;

        void set_state(uint8_t newstate)
        {
            state = newstate;
            apply_state();
        }

        uint8_t current_state()
        {
            return state;
        }
    };
}

#endif
