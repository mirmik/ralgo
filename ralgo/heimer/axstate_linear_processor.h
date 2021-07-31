/** @file */

#ifndef RALGO_HEIMER_AXSTATE_LINEAR_PROCESSOR_H
#define RALGO_HEIMER_AXSTATE_LINEAR_PROCESSOR_H

#include <igris/compiler.h>
#include <ralgo/heimer/axstate_signal_processor.h>

namespace heimer
{

    /**
        Преобразователь реализует линейное преобразование путём умножения сигнала на матрицу.

        R = M * L
        L = M^-1 * R
    */
    class axstate_linear_processor : public axstate_signal_processor
    {
//  struct signal_processor proc;
    public:
        float * matrix = nullptr;
        float * invert_matrix = nullptr;

    public:
        axstate_linear_processor() = default;
        axstate_linear_processor(const char * name, int dim);

        int dim();

        int feedback(disctime_t time) override;
        int serve(disctime_t time) override;
        int command(int argc, char ** argv, char * output, int outmax) override;
        void deinit() override;

        void on_activate(disctime_t) override;

        void init(
            const char * name,
            int dim,
            struct axis_state ** leftside,
            struct axis_state ** rightside,
            float * matrix,
            float * invert_matrix
        );

        void evaluate_invertion();
        void allocate_resources();
    };
}

#endif