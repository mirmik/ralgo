/**
    @file
*/

#ifndef RALGO_FILTER_APERIODIC_FILTER_H
#define RALGO_FILTER_APERIODIC_FILTER_H

#include <ralgo/filter/inout.h>

namespace ralgo
{
    template <class T> class aperiodic_filter : public ralgo::inout<T, T>
    {
        double koeff = 0;
        double timeconst = 1;
        double invert_timeconst = 1;
        T state = {};

    public:
        aperiodic_filter() {}

        aperiodic_filter(double _koeff) : koeff(_koeff) {}

        aperiodic_filter(double delta, double time_constant)
        {
            set_koefficient(delta, time_constant);
        }

        T serve(const T &in)
        {
            state += (in - state) * koeff;
            return state;
        }

        T serve(const T &in, double delta)
        {
            state += (in - state) * (delta / timeconst);
            return state;
        }

        T operator()(const T &in) override { return serve(in); }

        void reset(T val) { state = val; }

        aperiodic_filter &set_koefficient(double delta, double time_constant)
        {
            koeff = delta / time_constant;
            timeconst = time_constant;
            invert_timeconst = 1 / time_constant;
            return *this;
        }

        aperiodic_filter &set_timeconst(double time_constant)
        {
            timeconst = time_constant;
            return *this;
        }
    };
}

#endif
