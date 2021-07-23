#ifndef RALGO_TRAJECTORY_H
#define RALGO_TRAJECTORY_H

#include <ralgo/trajectory/tsdeform.h>
#include <ralgo/heimer2/heimer_types.h>
#include <ralgo/disctime.h>

typedef int (* trajectory_attime_t) (
    void * priv,
    disctime_t timestamp, position_t * outpos, position_t * outspd);

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