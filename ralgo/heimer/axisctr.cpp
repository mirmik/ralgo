#include <ralgo/heimer/axisctr.h>
#include <ralgo/heimer/sigtypes.h>
#include <ralgo/clinalg/vecops.h>
#include <ralgo/log.h>

#include <igris/math.h>
#include <igris/util/bug.h>
#include <igris/datastruct/sparse_array.h>

#include <string.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

using namespace heimer;

axis_controller::axis_controller(
    const char * name,
    struct axis_settings * setings,
    int dim
)
{
	init(name, setings, dim);
}

void axis_controller::set_handlers(
    void * operation_handlers_priv,
    void (* operation_start_handler)(void * priv, axis_controller * ax),
    void (* operation_finish_handler)(void * priv, axis_controller * ax)
)
{
	this->operation_handlers_priv = operation_handlers_priv;
	this->operation_start_handler = operation_start_handler;
	this->operation_finish_handler = operation_finish_handler;
}

void axis_controller::set_gain(double * gain)
{
	for (int i = 0; i < dim; ++i)
	{
		settings[i].gain = gain[i];
	}
}

void axis_controller::set_velocity_external(float speed)
{
	vel = speed / discrete_time_frequency();
}

void axis_controller::set_accdcc_external(float acc, float dcc)
{
	this->acc = acc / discrete_time_frequency() / discrete_time_frequency();
	this->dcc = dcc / discrete_time_frequency() / discrete_time_frequency();
}

void axis_controller::set_limits_external(double * back, double * forw)
{
	for (int i = 0; i < dim; ++i)
	{
		settings[i].limits_enabled = 1;
		settings[i].backlim = back[i];
		settings[i].forwlim = forw[i];
	}
}

void axis_controller::set_controlled(struct axis_state ** state)
{
	for (int i = 0; i < dim; ++i)
	{
		settings[i].controlled = state[i];
		settings[i].controlled->get();
	}
}

void axis_controller::finish_trajectory(disctime_t time, position_t * ctrpos)
{
	f.operation_finished_flag = 1;
	if (operation_finish_handler)
		operation_finish_handler(operation_handlers_priv, this);
	line_trajectory_set_point_hold(&lintraj, time, ctrpos);
	curtraj = &lintraj.traj;
	f.release_control_flag = 1;
}

int axis_controller::feedback(disctime_t time)
{
	(void) time;
	return 0;
}

int axis_controller::serve(disctime_t time)
{
	position_t ctrpos[dim];
	velocity_t ctrvel[dim];

	if (f.release_control_flag)
	{
		f.release_control_flag = 0;
		deactivate();
		return SIGNAL_PROCESSOR_RETURN_NOT_ACTIVE;
	}

	for (int i = 0; i < dim; ++i)
	{
		if (
		    !settings[i].controlled->current_controller ||
		    settings[i].controlled->current_controller != this
		)
		{
			return SIGNAL_PROCESSOR_RETURN_NOT_ACTIVE;
		}
	}

	if (!curtraj)
		return SIGNAL_PROCESSOR_RETURN_NOT_ACTIVE;

	int sts = curtraj->attime(curtraj, time, ctrpos, ctrvel);
	for (int i = 0; i < dim; ++i)
	{
		// Установить текущие целевые параметры.
		settings[i].controlled->ctrpos = ctrpos[i];
		settings[i].controlled->ctrvel = ctrvel[i];
	}

	if (sts && !f.operation_finished_flag)
	{
		finish_trajectory(time, ctrpos);
	}

	return 0;
}

int axis_controller::_absmove(
    disctime_t curtim,
    position_t * curpos,
    position_t * tgtpos,
    double extdist)
{
	for (int i = 0; i < dim; ++i)
		if (settings[i].controlled->current_controller)
		{
			return -1;
		}

	if (activate())
		return -1;


	position_t dist = vecops_distance_d(tgtpos, curpos, dim);
	disctime_t tgttim = curtim + (float)(ABS(extdist)) / vel;

	if (dist == 0 || vel == 0)
	{
		finish_trajectory(curtim, curpos);
		return 0;
	}

	disctime_t acc_time = (vel / acc);
	disctime_t dcc_time = (vel / dcc);

	line_trajectory_init_nominal_speed(&lintraj,
	                                   curtim,
	                                   tgttim,
	                                   curpos,
	                                   tgtpos,
	                                   acc_time,
	                                   dcc_time,
	                                   f.spattern_enabled
	                                  );

	f.operation_finished_flag = 0;
	f.release_control_flag = 0;

	if (operation_start_handler)
		operation_start_handler(operation_handlers_priv, this);
	curtraj = &lintraj.traj;
	return 0;
}


int axis_controller::incmove(disctime_t current_time, double * dist_real)
{
	position_t curpos[dim];
	position_t tgtpos[dim];
	double extdist = 0;

	for (int i = 0; i < dim; ++i)
	{
		extdist += dist_real[i] * dist_real[i];
		position_t dist = dist_real[i] * settings[i].gain;

		curpos[i] = settings[i].controlled->ctrpos;
		tgtpos[i] = curpos[i] + dist;

		if (settings[i].limits_enabled)
			tgtpos[i] = CLAMP(tgtpos[i],
			                  settings[i].backlim * settings[i].gain,
			                  settings[i].forwlim * settings[i].gain);
	}

	return _absmove(current_time, curpos, tgtpos, sqrt(extdist));
}

int axis_controller::absmove(disctime_t current_time, double * pos_real)
{

	position_t curpos[dim];
	position_t tgtpos[dim];
	double extdist = 0;

	for (int i = 0; i < dim; ++i)
	{
		double diff = pos_real[i] - settings[i].controlled->ctrpos / settings[i].gain;
		extdist += diff * diff;

		curpos[i] = settings[i].controlled->ctrpos;
		tgtpos[i] = pos_real[i] * settings[i].gain;

		if (settings[i].limits_enabled)
			tgtpos[i] = CLAMP(tgtpos[i],
			                  settings[i].backlim * settings[i].gain,
			                  settings[i].forwlim * settings[i].gain);
	}

	return _absmove(current_time, curpos, tgtpos, sqrt(extdist));
}

float axis_controller::ctrpos_external(int axno)
{
	return  settings[axno].controlled->ctrpos / settings[axno].gain;
}

float axis_controller::feedpos_external(int axno)
{
	return  settings[axno].controlled->feedpos / settings[axno].gain;
}

float axis_controller::ctrvel_external(int axno)
{
	return settings[axno].controlled->ctrvel * discrete_time_frequency() / settings[axno].gain;
}

axis_controller * heimer::create_axis_controller(const char * name, int dim)
{
	axis_controller * ptr = new axis_controller;
	struct axis_settings * settings = (struct axis_settings *) malloc(sizeof(struct axis_settings) * dim);
	ptr->init(name, settings, dim);
	ptr->f.dynamic_resources = 1;
	return ptr;
}

void axis_controller::release_controlled()
{
	for (int i = 0; i < dim; ++i)
	{
		if (settings[i].controlled)
		{
			settings[i].controlled->put();
			settings[i].controlled = NULL;
		}
	}
}

void axis_controller::deinit()
{
	release_controlled();
}

signal_head * axis_controller::iterate_left(signal_head * iter)
{
	if (iter == NULL)
		return settings[0].controlled;

	for (int i = 0; i < dim - 1; ++i)
	{
		if (iter == settings[i].controlled)
		{
			return settings[i + 1].controlled;
		}
	}

	return NULL;
}

signal_head * axis_controller::iterate_right(signal_head * iter)
{
	(void) iter;
	return NULL;
}


void axis_settings_init(struct axis_settings * settings)
{
	settings->controlled = NULL;
	settings->backlim = 0;
	settings->forwlim = 0;
	settings->sfpos.spos = 0;
	settings->sfpos.fpos = 0;
	settings->gain = 1;
	settings->limits_enabled = 0;
}

void axis_controller::init(
    const char * name,
    struct axis_settings * settings,
    int dim
)
{
	signal_processor::init(name);

	vel = 0;
	acc = 0;
	dcc = 0;

	for (int i = 0; i < dim; ++i)
		axis_settings_init(&settings[i]);
	this->settings = settings;
	this->dim = dim;

	flags = 0;

	operation_start_handler = NULL;
	operation_finish_handler = NULL;
	operation_handlers_priv = NULL;

	approvals = NULL;
	approvals_total = 0;

	curtraj = NULL;

	line_trajectory_init(&lintraj, 1,
	                     &settings[0].sfpos,
	                     dim
	                    );
}
