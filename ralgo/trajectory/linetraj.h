#ifndef RALGO_TRAJECTORY_LINETRAJ_H
#define RALGO_TRAJECTORY_LINETRAJ_H

#include <ralgo/trajectory/trajectory.h>

/// Простейшая линейная в пространстве параметров траектория.
struct line_trajectory
{
	struct trajectory traj;

	disctime_t stim;
	disctime_t ftim;

	int64_t * spos;
	int64_t * fpos;

	struct trajectory_speed_deformer tsd;
};

__BEGIN_DECLS

/// detail: Процедура не учитывает возможные начальную и оконечную скорости.
void line_trajectory_init_nominal_speed(
    struct line_trajectory * lintraj,
    int dim,
    disctime_t   stim,
    disctime_t   ftim,
    int64_t * spos,
    int64_t * fpos,

    disctime_t acc_time,
    disctime_t dcc_time,

    int full_spattern
);


int line_trajectory_attime (void * priv,
                            disctime_t time, 
                            struct control_phase * outs);

__END_DECLS

#endif