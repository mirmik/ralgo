#include <ralgo/heimer2/axisctr.h>
#include <stddef.h>
#include <stdio.h>

void axis_controller_init(struct axis_controller * axctr)
{
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
	axctr->vel = speed * DISTANCE_MULTIPLIER / discrete_time_frequency();
}

void axis_controller_set_accdcc_internal(struct axis_controller * axctr, float acc, float dcc)
{
	axctr->acc = acc;
	axctr->dcc = dcc;
}

void axis_controller_set_accdcc_external(struct axis_controller * axctr, float acc, float dcc)
{
	axctr->acc = acc * DISTANCE_MULTIPLIER / discrete_time_frequency() / discrete_time_frequency();
	axctr->dcc = dcc * DISTANCE_MULTIPLIER / discrete_time_frequency() / discrete_time_frequency();
}

void axis_controller_set_limits_external(struct axis_controller * axctr, double back, double forw)
{
	axctr->backlim = distance_float_to_fixed(back * axctr->gain);
	axctr->forwlim = distance_float_to_fixed(forw * axctr->gain);
}

void axis_controller_set_controlled(struct axis_controller * axctr, struct axis_state * state)
{
	axctr->controlled = state;
}


void axis_controller_serve(struct axis_controller * axctr, disctime_t time)
{
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
		axctr->operation_finished_flag = 1;
		axctr->operation_finish_handler(axctr->operation_handlers_priv, axctr);
		line_trajectory_set_point_hold(&axctr->lintraj, time, &ctrpos);
		axctr->curtraj = &axctr->lintraj.traj;
	}
}