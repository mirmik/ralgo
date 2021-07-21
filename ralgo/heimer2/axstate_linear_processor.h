#ifndef RALGO_HEIMER_AXSTATE_LINEAR_PROCESSOR_H
#define RALGO_HEIMER_AXSTATE_LINEAR_PROCESSOR_H

#include <igris/compiler.h>
#include <ralgo/heimer2/signal_processor.h>
#include <ralgo/heimer2/axis_state.h>

struct axstate_linear_processor
{
	struct signal_processor proc;

	int dim;

	struct axis_state ** leftside;
	struct axis_state ** rightside;

	float * matrix;
	float * invert_matrix;
};

__BEGIN_DECLS

void axstate_linear_processor_init(
    struct axstate_linear_processor * lproc,
    const char * name,
    int dim,
    struct axis_state ** leftside,
    struct axis_state ** rightside,
    float * matrix,
    float * invert_matrix
);

__END_DECLS

#endif