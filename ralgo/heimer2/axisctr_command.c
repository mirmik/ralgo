#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <ralgo/heimer2/axisctr.h>
#include <ralgo/heimer2/sigtypes.h>

static inline
int bind(struct axis_controller * axctr, int argc, char ** argv, char * output, int outmax)
{
	axis_controller_release_controlled(axctr);

	struct signal_head * sig = signal_get_by_name(argv[1]);
	if (!sig)
		return -1;

	if (sig->type != SIGNAL_TYPE_AXIS_STATE)
	{
		snprintf(output, outmax, "Wrong signal type");
		return -1;
	}

	axis_controller_set_controlled(axctr, mcast_out(sig, struct axis_state, sig));
	return 0;
}

static inline
int info(struct axis_controller * axctr, int argc, char ** argv, char * output, int outmax)
{
//	if (!axctr->controlled) 
//	{
//		snprintf(output, outmax, "Constrolled axis is not binded");
//		return 0; 		
//	}

/*	int64_t ctrpos_int = axctr->controlled->ctrpos; 
	int64_t feedpos_int = axctr->controlled->feedpos;

	float ctrpos_ext =  axis_controller_ctrpos_external(axctr); 
	float feedpos_ext = axis_controller_feedpos_external(axctr);

	float ctrvel = axctr->controlled->ctrvel; 
	float feedvel= axctr->controlled->feedvel;

	snprintf(output, outmax, 
		"ctrpos_ext :%f\r\n"
		"ctrpos_int :%ld\r\n" 
		"feedpos_ext:%f\r\n"
		"feedpos_int:%ld\r\n"
		"ctrvel     :%f\r\n"
		"feedvel    :%f\r\n",
		ctrpos_ext,
		ctrpos_int,
		feedpos_ext,
		feedpos_int,
		ctrvel,
		feedvel
	);
*/
	return 0;
}

static inline
int incmov(struct axis_controller * axctr, int argc, char ** argv, char * output, int outmax)
{
	int dim = axctr->dim;
	position_t dist[dim];

	for (int i = 0; i < dim; ++i) 
	{
		dist[i] = atof(argv[i]);
	}

	return axis_controller_incmove(axctr, discrete_time(), dist);
}

static inline
int absmov(struct axis_controller * axctr, int argc, char ** argv, char * output, int outmax)
{
	int dim = axctr->dim;
	position_t pos[dim];

	for (int i = 0; i < dim; ++i) 
	{
		pos[i] = atof(argv[i]);
	}
	return axis_controller_absmove(axctr, discrete_time(), pos);
}

int axis_controller_command(struct signal_processor * sigproc, int argc, char ** argv, char * output, int outmax)
{
	int status = ENOENT;
	struct axis_controller * axctr = mcast_out(sigproc, struct axis_controller, sigproc);

	if (strcmp("bind", argv[0]) == 0)
		status = bind(axctr, argc - 1, argv + 1, output, outmax);

	else if (strcmp("info", argv[0]) == 0)
		status = info(axctr, argc - 1, argv + 1, output, outmax);

	else if (strcmp("absmov", argv[0]) == 0)
		status = absmov(axctr, argc - 1, argv + 1, output, outmax);

	else if (strcmp("incmov", argv[0]) == 0)
		status = incmov(axctr, argc - 1, argv + 1, output, outmax);

	return status;
}
