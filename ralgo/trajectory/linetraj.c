#include <ralgo/trajectory/linetraj.h>

int line_trajectory_attime (void * priv,
                            disctime_t time,
                            position_t  * pos,
                            velocity_t * spd)
{
	struct line_trajectory * traj = (struct line_trajectory *) priv;

	// Умножение на коэффициент времени перерасщитывает скорость
	// взятую на дискретную единицу времени в скорость взятую
	// на единицу времени рабочего пространства.

	int local_time = time - traj->stim;
	int full_time = traj->ftim - traj->stim;

	assert(full_time > 0);

	float time_unit = local_time <= 0 ? 0 : (float)(local_time) / (float)(full_time);

	float posmod = tsdeform_posmod(&traj->tsd, time_unit);
	float spdmod = tsdeform_spdmod(&traj->tsd, time_unit);

	for (unsigned int i = 0; i < traj->traj.dim; ++i)
	{
		// Положение вычисляется по формуле линейной интерполяции.
		pos[i] = traj->fpos[i] * posmod + traj->spos[i] * (1 - posmod);

		// Скорость вычисляется просто путём умножения на коэффицент.
		// Выходная скорость имеет размерность единицы длины
		// на дискретную единицу времени
		velocity_t naive_speed = (velocity_t)(traj->fpos[i] - traj->spos[i]) / (velocity_t)full_time;
		spd[i] = naive_speed * spdmod;
	}

	return (tsdeform_is_finished(&traj->tsd, time_unit) ||
	        traj->stim == traj->ftim) ? 1 : 0;
}

void line_trajectory_init(struct line_trajectory * lintraj, int dim, position_t * spos, position_t * fpos)
{
	trajectory_init(&lintraj->traj, dim, line_trajectory_attime);
	lintraj->spos = spos;
	lintraj->fpos = fpos;
}

/// detail: Процедура не учитывает возможные начальную и оконечную скорости.
void line_trajectory_init_nominal_speed(
    struct line_trajectory * lintraj,
    disctime_t   stim,
    disctime_t   ftim,
    position_t * spos,
    position_t * fpos,

    disctime_t acc_time,
    disctime_t dcc_time,

    int full_spattern
)
{
	lintraj->stim = stim;
	lintraj->ftim = ftim;

	for (unsigned int i = 0; i < lintraj->traj.dim; ++i)
	{
		lintraj->spos[i] = spos[i];
		lintraj->fpos[i] = fpos[i];
	}

	int time = ftim - stim;

	float acc_part = (float)acc_time / (float)time;
	float dcc_part = (float)dcc_time / (float)time;

	tsdeform_set_speed_pattern(
	    &lintraj->tsd,
	    acc_part, dcc_part,
	    0, 0,
	    full_spattern);
}

void line_trajectory_set_point_hold(
    struct line_trajectory * traj,
    disctime_t ftim,
    position_t * pos)
{
	traj->ftim = ftim;
	traj->stim = ftim - 1;

	for (unsigned int i = 0; i < traj->traj.dim; ++i)
	{
		traj->spos[i] = pos[i];
		traj->fpos[i] = pos[i];
	}

	tsdeform_set_stop_pattern(&traj->tsd);
}