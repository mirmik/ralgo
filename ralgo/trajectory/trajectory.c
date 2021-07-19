#include <ralgo/trajectory/trajectory.h>

void trajectory_init(
	struct trajectory * traj,
	int dim,
	trajectory_attime_t attime
) 
{
	traj->dim = dim;
	traj->attime = attime;
}
