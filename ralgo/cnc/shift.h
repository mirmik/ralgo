#ifndef RALGO_SHIFT_H
#define RALGO_SHIFT_H

#include <igris/container/static_vector.h>

typedef uint16_t revolver_t;

namespace cnc
{
    class control_shift
    {
    public:
        revolver_t step = {};
        revolver_t direction = {};
        igris::static_vector<double, NMAX_AXES> _velocity = {};

    public:
        control_shift() {}

        control_shift(revolver_t step, revolver_t direction)
            : step(step), direction(direction)
        {
        }

        control_shift(revolver_t step,
                      revolver_t direction,
                      double *velocity,
                      int total_axes)
            : step(step), direction(direction)
        {
            for (int i = 0; i < total_axes; i++)
            {
                _velocity[i] = velocity[i];
            }
        }

        igris::static_vector<double, NMAX_AXES> const &velocity()
        {
            return _velocity;
        }
    };
}

#endif
