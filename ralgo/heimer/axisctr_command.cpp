#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <ralgo/heimer/axisctr.h>
#include <ralgo/heimer/sigtypes.h>

using namespace heimer;

static inline
int setvel(axis_controller * axctr, int argc, char ** argv, char * output, int outmax)
{
	if (argc < 1)
	{
		snprintf(output, outmax, "Need argument");
		return -1;
	}

	axctr->set_velocity_external(atof(argv[0]));
	return 0;
}

static inline
int setacc(axis_controller * axctr, int argc, char ** argv, char * output, int outmax)
{
	if (argc < 1)
	{
		snprintf(output, outmax, "Need argument");
		return -1;
	}

	axctr->set_acceleration_external(atof(argv[0]));

	if (argc > 1)
		axctr->set_decceleration_external(atof(argv[1]));
	return 0;
}

static inline
int setdcc(axis_controller * axctr, int argc, char ** argv, char * output, int outmax)
{
	if (argc < 1)
	{
		snprintf(output, outmax, "Need argument");
		return -1;
	}

	axctr->set_acceleration_external(atof(argv[0]));
	return 0;
}

static inline
int bind(axis_controller * axctr, int argc, char ** argv, char * output, int outmax)
{
	if (argc != axctr->dim)
	{
		snprintf(output, outmax, "Can't bind %d symbols for %d dim axisctr", argc, axctr->dim);
		return -1;
	}

	{
		axis_state * arr[argc];

		for (int i = 0; i < argc; ++i)
		{
			signal_head * sig = signal_get_by_name(argv[i]);

			if (!sig)
			{
				snprintf(output, outmax, "Wrong signal name '%s' (type 'siglist' for display)", argv[i]);
				return -1;
			}

			if (sig->type != SIGNAL_TYPE_AXIS_STATE)
			{
				snprintf(output, outmax, "Wrong signal type. name:(%s)", sig->name);
				return -1;
			}

			arr[i] = static_cast<axis_state *>(sig);
		}

		axctr->release_controlled();
		axctr->set_controlled(arr);
	}

	return 0;
}

static inline
int setgain(axis_controller * axctr, int argc, char ** argv, char * output, int outmax)
{
	if (argc != axctr->dim)
	{
		snprintf(output, outmax, "Can't bind %d symbols for %d dim axisctr", argc, axctr->dim);
		return -1;
	}

	for (int i = 0; i < argc; ++i)
	{
		axctr->settings[i].gain = atof(argv[i]);
	}

	return 0;
}

static inline
int info(axis_controller * axctr, int argc, char ** argv, char * output, int outmax)
{
	(void) argc;
	(void) argv;

	int bufsize = 96;
	char buf[bufsize];

	memset(buf, 0, bufsize);
	snprintf(buf, bufsize, "isactive:%d, ", axctr->is_active());
	strncat(output, buf, outmax);

	memset(buf, 0, bufsize);
	snprintf(buf, bufsize, "dim:%d, ", axctr->dim);
	strncat(output, buf, outmax);

	memset(buf, 0, bufsize);
	snprintf(buf, bufsize, "signals:");
	strncat(output, buf, outmax);

	memset(buf, 0, bufsize);
	for (int i = 0; i < axctr->dim - 1; ++i)
	{
		snprintf(buf, bufsize, "%s,", axctr->settings[i].controlled->name);
		strncat(output, buf, outmax);
	}
	snprintf(buf, bufsize, "%s\r\n", axctr->settings[axctr->dim - 1].controlled->name);
	strncat(output, buf, outmax);

	memset(buf, 0, bufsize);
	snprintf(buf, bufsize, "external: vel:%f, acc:%f, dcc:%f\r\n", 
		axctr->external_velocity(), axctr->external_acceleration(), axctr->external_decceleration());
	strncat(output, buf, outmax);

	memset(buf, 0, bufsize);
	snprintf(buf, bufsize, "flags: opfinished:%d, releaseflag:%d, dynamic:%d, spattern:%d\r\n",
	         axctr->f.operation_finished_flag,
	         axctr->f.release_control_flag,
	         axctr->f.dynamic_resources,
	         (uint8_t)axctr->f.spattern_enabled);
	strncat(output, buf, outmax);

	for (int i = 0; i < axctr->dim; ++i) 
	{
		memset(buf, 0, bufsize);
		snprintf(buf, bufsize, "axsets: lims: %d,%f,%f sfpos: %f,%f gain: %f\r\n", 
			axctr->settings[i].limits_enabled,
			axctr->settings[i].backlim,
			axctr->settings[i].forwlim,
			axctr->settings[i].sfpos.spos,
			axctr->settings[i].sfpos.fpos,
			axctr->settings[i].gain
		);
	strncat(output, buf, outmax);
	}

	return 0;
}

static inline
int incmov(axis_controller * axctr, int argc, char ** argv, char * output, int outmax)
{
	if (argc != axctr->dim)
	{
		snprintf(output, outmax, "Can't use %d coord for %d dim axisctr", argc, axctr->dim);
		return -1;
	}

	int dim = axctr->dim;
	position_t dist[dim];

	for (int i = 0; i < dim; ++i)
	{
		dist[i] = atof(argv[i]);
	}

	return axctr->incmove(discrete_time(), dist);
}

static inline
int absmov(axis_controller * axctr, int argc, char ** argv, char * output, int outmax)
{
	if (argc != axctr->dim)
	{
		snprintf(output, outmax, "Can't use %d coord for %d dim axisctr", argc, axctr->dim);
		return -1;
	}

	int dim = axctr->dim;
	position_t pos[dim];

	for (int i = 0; i < dim; ++i)
	{
		pos[i] = atof(argv[i]);
	}
	return axctr->absmove(discrete_time(), pos);
}

static inline
int stop(axis_controller * axctr, int, char **, char *, int)
{
	return axctr->stop(discrete_time());
}

static inline
int hardstop(axis_controller * axctr, int, char **, char *, int)
{
	return axctr->hardstop(discrete_time());
}

int axis_controller::command(int argc, char ** argv, char * output, int outmax)
{
	int status = ENOENT;

	if (strcmp("bind", argv[0]) == 0)
		status = bind(this, argc - 1, argv + 1, output, outmax);

	else if (strcmp("info", argv[0]) == 0)
		status = info(this, argc - 1, argv + 1, output, outmax);

	else if (strcmp("absmove", argv[0]) == 0)
		status = absmov(this, argc - 1, argv + 1, output, outmax);

	else if (strcmp("incmove", argv[0]) == 0)
		status = incmov(this, argc - 1, argv + 1, output, outmax);

	else if (strcmp("stop", argv[0]) == 0)
		status = ::stop(this, argc - 1, argv + 1, output, outmax);

	else if (strcmp("hardstop", argv[0]) == 0)
		status = ::hardstop(this, argc - 1, argv + 1, output, outmax);

	else if (strcmp("setvel", argv[0]) == 0)
		status = setvel(this, argc - 1, argv + 1, output, outmax);

	else if (strcmp("setacc", argv[0]) == 0)
		status = setacc(this, argc - 1, argv + 1, output, outmax);

	else if (strcmp("setdcc", argv[0]) == 0)
		status = setdcc(this, argc - 1, argv + 1, output, outmax);

	else if (strcmp("setgain", argv[0]) == 0)
		status = setgain(this, argc - 1, argv + 1, output, outmax);

	return status;
}
