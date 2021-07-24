#include <ralgo/heimer2/axisctr.h>
#include <ralgo/heimer2/sigtypes.h>
#include <igris/math.h>
#include <igris/util/bug.h>

#include <string.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

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

void axis_controller_set_gain(struct axis_controller * axctr, double gain)
{
	axctr->gain = gain;
}

void axis_controller_set_limits_internal(struct axis_controller * axctr, position_t back, position_t forw)
{
	axctr->backlim = back;
	axctr->forwlim = forw;
}

void axis_controller_set_velocity_internal(struct axis_controller * axctr, velocity_t speed)
{
	axctr->vel = speed;
}

void axis_controller_set_velocity_external(struct axis_controller * axctr, float speed)
{
	axctr->vel = speed / discrete_time_frequency() * axctr->gain;
}

void axis_controller_set_accdcc_internal(struct axis_controller * axctr, velocity_t  acc, velocity_t  dcc)
{
	axctr->acc = acc;
	axctr->dcc = dcc;
}

void axis_controller_set_accdcc_external(struct axis_controller * axctr, float acc, float dcc)
{
	axctr->acc = acc / discrete_time_frequency() / discrete_time_frequency() * axctr->gain;
	axctr->dcc = dcc / discrete_time_frequency() / discrete_time_frequency() * axctr->gain;
}

void axis_controller_set_limits_external(struct axis_controller * axctr, double back, double forw)
{
	axctr->backlim = back * axctr->gain;
	axctr->forwlim = forw * axctr->gain;
}

void axis_controller_set_controlled(struct axis_controller * axctr, struct axis_state * state)
{
	axctr->controlled = state;

	signal_head_get(&axctr->controlled->sig);
}

void axis_controller_finish_trajectory(struct axis_controller * axctr, disctime_t time, position_t ctrpos)
{
	axctr->operation_finished_flag = 1;
	axctr->operation_finish_handler(axctr->operation_handlers_priv, axctr);
	line_trajectory_set_point_hold(&axctr->lintraj, time, &ctrpos);
	axctr->curtraj = &axctr->lintraj.traj;
}

void axis_controller_feedback(struct signal_processor * sigproc, disctime_t time)
{
	//pass
}

void axis_controller_serve(struct signal_processor * sigproc, disctime_t time)
{
	struct axis_controller * axctr =  mcast_out(sigproc, struct axis_controller, sigproc);
	position_t ctrpos;
	velocity_t ctrvel;

	if (axctr->controlled->sig.current_controller && axctr->controlled->sig.current_controller != sigproc) 
		return;

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
    position_t curpos,
    position_t tgtpos)
{
	if (axctr->controlled->sig.current_controller) 
	{
		return -1;
	}

	if (signal_processor_activate(&axctr->sigproc))
		return -1;
	
	position_t dist = tgtpos - curpos;
	disctime_t tgttim = curtim + (float)(ABS(dist)) / axctr->vel;

	if (dist == 0 || axctr->vel == 0)
	{
		axis_controller_finish_trajectory(axctr, curtim, curpos);
		return 0;
	}

	disctime_t acc_time = (axctr->vel / axctr->acc);
	disctime_t dcc_time = (axctr->vel / axctr->dcc);

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
	position_t dist = dist_real * axctr->gain;

	position_t curpos = axctr->controlled->ctrpos;
	position_t tgtpos = curpos + dist;

	tgtpos = CLAMP(tgtpos,
	               axctr->backlim,
	               axctr->forwlim);

	return __axis_controller_absmove(axctr, current_time, curpos, tgtpos);
}

int axis_controller_absmove(struct axis_controller * axctr, disctime_t current_time, double pos_real)
{
	position_t curpos = axctr->controlled->ctrpos;
	position_t tgtpos = tgtpos * axctr->gain;

	tgtpos = CLAMP(tgtpos,
	               axctr->backlim,
	               axctr->forwlim);

	return __axis_controller_absmove(axctr, current_time, curpos, tgtpos);
}

float axis_controller_ctrpos_external(struct axis_controller * axctr)
{
	return  axctr->controlled->ctrpos / axctr->gain;
}

float axis_controller_feedpos_external(struct axis_controller * axctr)
{
	return  axctr->controlled->feedpos / axctr->gain;
}

float axis_controller_ctrvel_external(struct axis_controller * axctr)
{
	return axctr->controlled->ctrvel * discrete_time_frequency() / axctr->gain;
}

struct axis_controller * create_axis_controller(const char * name)
{
	struct axis_controller * ptr = (struct axis_controller *) malloc(sizeof(struct axis_controller));
	axis_controller_init(ptr, name);
	return ptr;
}

void axis_controller_release_controlled(struct axis_controller * axctr)
{
	if (axctr->controlled)
	{
		signal_head_put(&axctr->controlled->sig);
		axctr ->controlled = NULL;
	}
}

void axis_controller_deinit(struct signal_processor * sigproc)
{
	struct axis_controller * axctr = mcast_out(sigproc, struct axis_controller, sigproc);
	axis_controller_release_controlled(axctr);
}

struct signal_head * axis_controller_iterate_left(struct signal_processor * sigproc, struct signal_head * iter)
{
	struct axis_controller * axctr = mcast_out(sigproc, struct axis_controller, sigproc);

	if (iter == NULL)
		return &axctr->controlled->sig;

	else
		return NULL;
}

const struct signal_processor_operations axisctr_ops =
{
	.feedback = axis_controller_feedback,
	.serve = axis_controller_serve,
	.command = axis_controller_command,
	.deinit = axis_controller_deinit,
	.iterate_left = axis_controller_iterate_left
};

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
	                     &axctr->_line_trajectory_sfpos,
	                     0
	                    );
}
