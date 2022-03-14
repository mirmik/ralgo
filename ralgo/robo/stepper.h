#ifndef RALGO_ROBO_STEPPER_H
#define RALGO_ROBO_STEPPER_H

namespace robo
{
    class stepper
    {
    protected:
        int64_t counter = 0;
        bool _simulator_mode = false;

    public:
        virtual void inc() { ++counter; }
        virtual void dec() { --counter; }
        virtual ~stepper() = default;

        int64_t steps_count() { return counter; }

        void set_counter_value(int64_t val) { counter = val; }
        void simulator_mode(bool en) 
        {
            _simulator_mode = en;
        }
    };
}

#endif
