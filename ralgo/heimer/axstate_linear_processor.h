/** @file */

#ifndef RALGO_HEIMER_AXSTATE_LINEAR_PROCESSOR_H
#define RALGO_HEIMER_AXSTATE_LINEAR_PROCESSOR_H

#include <igris/compiler.h>
#include <ralgo/heimer/signal_processor.h>
#include <ralgo/heimer/axis_state.h>

/**
    Преобразователь реализует линейное преобразование путём умножения сигнала на матрицу.

    R = M * L
    L = M^-1 * R
*/
class axstate_linear_processor : public signal_processor
{
//  struct signal_processor proc;
private:
    int dim;

    struct axis_state ** leftside;
    struct axis_state ** rightside;

    float * matrix;
    float * invert_matrix;

public:
    int feedback(disctime_t time) override;
    int serve(disctime_t time) override;
    int command(int argc, char ** argv, char * output, int outmax) override;
    void deinit() override;
    struct signal_head * iterate_left(struct signal_head *) override;
    struct signal_head * iterate_right(struct signal_head *) override;

    void init(
        const char * name,
        int dim,
        struct axis_state ** leftside,
        struct axis_state ** rightside,
        float * matrix,
        float * invert_matrix
    );

    void evaluate_invertion();
};

#endif