#ifndef RALGO_TRAJECTORY_LINETRAJ_H
#define RALGO_TRAJECTORY_LINETRAJ_H

#include <ralgo/trajectory/trajectory.h>
#include <ralgo/heimer2/heimer_types.h>

/// Простейшая линейная в пространстве параметров траектория.
struct line_trajectory
{
	struct trajectory traj;

	disctime_t stim;
	disctime_t ftim;

	position_t * spos;
	position_t * fpos;

	struct trajectory_speed_deformer tsd;
};

__BEGIN_DECLS

void line_trajectory_init(struct line_trajectory * lintraj, int dim, position_t * spos, position_t * fpos);

/// detail: Процедура не учитывает возможные начальную и оконечную скорости.
void line_trajectory_init_nominal_speed(
    struct line_trajectory * lintraj,
    disctime_t   stim,
    disctime_t   ftim,
    position_t  * spos,
    position_t  * fpos,

    disctime_t acc_time,
    disctime_t dcc_time,

    int full_spattern
);


int line_trajectory_attime (void * priv,
                            disctime_t time, 
                            position_t  * outpos,
                            velocity_t * outvel);

void line_trajectory_set_point_hold(
    struct line_trajectory * lintraj, 
    disctime_t time,
    position_t * pos);

__END_DECLS

#endif