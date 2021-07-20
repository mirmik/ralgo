#include <ralgo/heimer2/axisctr.h>
#include <igris/math.h>

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

const struct signal_processor_operations axisctr_ops =
{
	.serve = axis_controller_serve
};

void axis_controller_set_handlers(
    struct axis_controller * axctr,
    void * operation_handlers_priv,
    void (* operation_start_handler)(void * priv, struct axis_controller * ax),
    void (* operation_finish_handler)(void * priv, struct axis_controller * ax)
)
{
	axctr->operation_handlers_priv = operation_handlers_priv;
	axctr->operation_start_handler = operation_start_handler;
	axctr->operation_finish_handler = operation_finish_handler;
}


void axis_controller_init(struct axis_controller * axctr, const char * name)
{
	signal_processor_init(&axctr->sigproc, name, &axisctr_ops);

	axctr->vel = 0;
	axctr->acc = 0;
	axctr->dcc = 0;

	axctr->gain = 0;

	axctr->backlim = 0;
	axctr->forwlim = 0;

	axctr->operation_finished_flag = 0;

	axctr->operation_start_handler = NULL;
	axctr->operation_finish_handler = NULL;
	axctr->operation_handlers_priv = NULL;

	axctr->spattern_enabled = 0;
	axctr->curtraj = NULL;
	axctr->controlled = NULL;

	line_trajectory_init(&axctr->lintraj, 1,
	                     &axctr->_line_trajectory_spos,
	                     &axctr->_line_trajectory_fpos
	                    );
}

void axis_controller_set_gain(struct axis_controller * axctr, double gain)
{
	axctr->gain = gain;
}

void axis_controller_set_limits_internal_fixed(struct axis_controller * axctr, int64_t back, int64_t forw)
{
	axctr->backlim = back;
	axctr->forwlim = forw;
}

void axis_controller_set_velocity_internal(struct axis_controller * axctr, float speed)
{
	axctr->vel = speed;
}

void axis_controller_set_velocity_external(struct axis_controller * axctr, float speed)
{
	axctr->vel = speed * DISTANCE_MULTIPLIER / discrete_time_frequency() * axctr->gain;
}

void axis_controller_set_accdcc_internal(struct axis_controller * axctr, float acc, float dcc)
{
	axctr->acc = acc;
	axctr->dcc = dcc;
}

void axis_controller_set_accdcc_external(struct axis_controller * axctr, float acc, float dcc)
{
	axctr->acc = acc * DISTANCE_MULTIPLIER / discrete_time_frequency() / discrete_time_frequency() * axctr->gain;
	axctr->dcc = dcc * DISTANCE_MULTIPLIER / discrete_time_frequency() / discrete_time_frequency() * axctr->gain;
}

void axis_controller_set_limits_external(struct axis_controller * axctr, double back, double forw)
{
	axctr->backlim = distance_float_to_fixed(back * axctr->gain);
	axctr->forwlim = distance_float_to_fixed(forw * axctr->gain);
}

void axis_controller_set_controlled(struct axis_controller * axctr, struct axis_state * state)
{
	axctr->controlled = state;

	signal_head_get(&axctr->controlled->sig);
}

void axis_controller_finish_trajectory(struct axis_controller * axctr, disctime_t time, int64_t ctrpos)
{
	axctr->operation_finished_flag = 1;
	axctr->operation_finish_handler(axctr->operation_handlers_priv, axctr);
	line_trajectory_set_point_hold(&axctr->lintraj, time, &ctrpos);
	axctr->curtraj = &axctr->lintraj.traj;
}

void axis_controller_serve(struct signal_processor * sigproc, disctime_t time)
{
	struct axis_controller * axctr =  mcast_out(sigproc, struct axis_controller, sigproc);
	int64_t ctrpos;
	float   ctrvel;

	if (!axctr->curtraj)
		return;

	// Установить текущие целевые параметры.
	int sts = axctr->curtraj->attime(axctr->curtraj, time, &ctrpos, &ctrvel);
	axctr->controlled->ctrpos = ctrpos;
	axctr->controlled->ctrvel = ctrvel;

	assert(!isnan(ctrvel));

	if (sts && !axctr->operation_finished_flag)
	{
		axis_controller_finish_trajectory(axctr, time, ctrpos);
	}
}

static
int __axis_controller_absmove(
    struct axis_controller * axctr,
    disctime_t curtim,
    int64_t curpos,
    int64_t tgtpos)
{
	int64_t dist = tgtpos - curpos;
	disctime_t tgttim = curtim + (float)(ABS(dist)) / axctr->vel;

	if (dist == 0 || axctr->vel == 0)
	{
		axis_controller_finish_trajectory(axctr, curtim, curpos);
		return 0;
	}

	float acc_time = axctr->vel / axctr->acc;
	float dcc_time = axctr->vel / axctr->dcc;

	line_trajectory_init_nominal_speed(&axctr->lintraj,
	                                   curtim,
	                                   tgttim,
	                                   &curpos,
	                                   &tgtpos,
	                                   acc_time,
	                                   dcc_time,
	                                   axctr->spattern_enabled
	                                  );

	axctr->operation_finished_flag = 0;
	if (axctr->operation_start_handler)
		axctr->operation_start_handler(axctr->operation_handlers_priv, axctr);
	axctr->curtraj = &axctr->lintraj.traj;
	return 0;
}


int axis_controller_incmove(struct axis_controller * axctr, disctime_t current_time, double dist_real)
{
	int64_t dist = distance_float_to_fixed(dist_real * axctr->gain);

	int64_t curpos = axctr->controlled->ctrpos;
	int64_t tgtpos = curpos + dist;

	tgtpos = CLAMP(tgtpos,
	               axctr->backlim,
	               axctr->forwlim);

	return __axis_controller_absmove(axctr, current_time, curpos, tgtpos);
}

int axis_controller_absmove(struct axis_controller * axctr, disctime_t current_time, double pos_real)
{
	int64_t curpos = axctr->controlled->ctrpos;
	int64_t tgtpos = distance_float_to_fixed(tgtpos * axctr->gain);

	tgtpos = CLAMP(tgtpos,
	               axctr->backlim,
	               axctr->forwlim);

	return __axis_controller_absmove(axctr, current_time, curpos, tgtpos);
}

float axis_controller_ctrpos_external(struct axis_controller * axctr)
{
	return  distance_fixed_to_float(axctr->controlled->ctrpos) / axctr->gain;
}

float axis_controller_ctrvel_external(struct axis_controller * axctr)
{
	return axctr->controlled->ctrvel * discrete_time_frequency() / DISTANCE_MULTIPLIER / axctr->gain;
}

struct axis_controller * create_axis_controller(struct axis_controller * axctr, const char * name)
{
	struct axis_controller * ptr = (struct axis_controller *) malloc(sizeof(struct axis_controller));
	axis_controller_init(ptr, name);
	return ptr;
}