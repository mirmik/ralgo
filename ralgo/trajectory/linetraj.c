#include <ralgo/trajectory/linetraj.h>

int line_trajectory_attime (void * priv,
                            disctime_t time, 
                            struct control_phase * outs)
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
		outs[i].pos = traj->fpos[i] * posmod + traj->spos[i] * (1 - posmod);

		// Скорость вычисляется просто путём умножения на коэффицент.
		// Выходная скорость имеет размерность единицы длины 
		// на дискретную единицу времени 
		float naive_speed = (float)(traj->fpos[i] - traj->spos[i]) / (float)full_time;
		outs[i].spd = naive_speed * spdmod;
	}

	return (tsdeform_is_finished(&traj->tsd, time_unit) || 
		traj->stim == traj->ftim) ? 1 : 0;
}

static
void line_trajectory_init(struct line_trajectory * lintraj, int dim) 
{
	trajectory_init(&lintraj->traj, dim, line_trajectory_attime);
}

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
)
{
	line_trajectory_init(lintraj, dim);

	lintraj->stim = stim;
	lintraj->ftim = ftim;
	lintraj->spos = spos;
	lintraj->fpos = fpos;

	int time = ftim - stim;

	float acc_part = (float)acc_time / (float)time;
	float dcc_part = (float)dcc_time / (float)time;

	tsdeform_set_speed_pattern(
	    &lintraj->tsd,
	    acc_part, dcc_part,
	    0, 0,
	    full_spattern);
}