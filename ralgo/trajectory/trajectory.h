#ifndef RALGO_TRAJECTORY_H
#define RALGO_TRAJECTORY_H

#include <ralgo/heimer2/phase.h>
#include <ralgo/trajectory/tsdeform.h>
#include <ralgo/disctime.h>

typedef int (* trajectory_attime_t) (
    void * priv,
    disctime_t timestamp, struct control_phase * outs);

struct trajectory 
{
	int dim;
	trajectory_attime_t attime;
};

__BEGIN_DECLS

void trajectory_init(
	struct trajectory * traj,
	int dim,
	trajectory_attime_t attime	
);

__END_DECLS

#endif