#ifndef RALGO_SHIFT_H
#define RALGO_SHIFT_H

typedef uint16_t revolver_t;

namespace cnc
{
    class control_shift
    {
    public:
        revolver_t step = {};
        revolver_t direction = {};
        float speed[NMAX_AXES];

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
                speed[i] = velocity[i];
            }
        }
    };
}

#endif
