#include <ralgo/heimer2/axstate_sincos_processor.h>
#include <ralgo/heimer2/distance.h>

void axstate_sincos_processor_serve(struct signal_processor * proc, disctime_t time)
{
	struct axstate_sincos_processor * scproc = (struct axstate_sincos_processor *) proc;

	struct axis_state * xl = scproc->leftside[0];
	struct axis_state * yl = scproc->leftside[1];
	struct axis_state * al = scproc->leftside[2];

	struct axis_state * xr = scproc->rightside[0];
	struct axis_state * yr = scproc->rightside[1];
	struct axis_state * ar = scproc->rightside[2];

	float ascale = scproc->alpha_to_radian_scale;

	position_t aposoff = ar->ctrpos + scproc->a_offset;  // ar + aoff
	float s = heimpos_sin(aposoff * ascale);             // sin((ar + aoff) * ascale)
	float c = heimpos_cos(aposoff * ascale);             // cos((ar + aoff) * ascale)
	float avelocity = ar->ctrvel * ascale;               // dar/dt * ascale

	xl->ctrpos = xr->ctrpos + scproc->x_offset + scproc->radius * c;
	yl->ctrpos = yr->ctrpos + scproc->y_offset + scproc->radius * s;
	al->ctrpos = ar->ctrpos;

	xl->ctrvel = xr->ctrvel - distance_fixed_to_float(scproc->radius) * s * avelocity; 
	yl->ctrvel = yr->ctrvel + distance_fixed_to_float(scproc->radius) * c * avelocity;
	al->ctrvel = ar->ctrvel;
}

void axstate_sincos_processor_feedback(struct signal_processor * proc, disctime_t time)
{
	struct axstate_sincos_processor * scproc = (struct axstate_sincos_processor *) proc;

	struct axis_state * xl = scproc->leftside[0];
	struct axis_state * yl = scproc->leftside[1];
	struct axis_state * al = scproc->leftside[2];

	struct axis_state * xr = scproc->rightside[0];
	struct axis_state * yr = scproc->rightside[1];
	struct axis_state * ar = scproc->rightside[2];

	float ascale = scproc->alpha_to_radian_scale;

	position_t aposoff = ar->ctrpos + scproc->a_offset;  // ar + aoff
	float s = heimpos_sin(aposoff * ascale);             // sin((ar + aoff) * ascale)
	float c = heimpos_cos(aposoff * ascale);             // cos((ar + aoff) * ascale)
	float avelocity = ar->ctrvel * ascale;               // dar/dt * ascale

	xl->ctrpos = xr->ctrpos + scproc->x_offset + scproc->radius * c;
	yl->ctrpos = yr->ctrpos + scproc->y_offset + scproc->radius * s;
	al->ctrpos = ar->ctrpos;

	xl->ctrvel = xr->ctrvel - scproc->radius * s * avelocity; 
	yl->ctrvel = yr->ctrvel + scproc->radius * c * avelocity;
	al->ctrvel = ar->ctrvel;
}

int  axstate_sincos_processor_command(struct signal_processor * proc, int argc, char ** argv, char * output, int outmax)
{
	struct axstate_sincos_processor * scproc = (struct axstate_sincos_processor *) proc;
}

void axstate_sincos_processor_deinit(struct signal_processor * proc)
{
	struct axstate_sincos_processor * scproc = (struct axstate_sincos_processor *) proc;
}

const struct signal_processor_operations axstate_sincos_processor_ops =
{
	.feedback = axstate_sincos_processor_feedback,
	.serve = axstate_sincos_processor_serve,
	.deinit = axstate_sincos_processor_deinit,
	.command = axstate_sincos_processor_command,
};

void axstate_sincos_processor_set_alpha_scale(
    struct axstate_sincos_processor * scproc,
    float ascale
)
{
	scproc->alpha_to_radian_scale = ascale;
}

void axstate_sincos_processor_set_offset(struct axstate_sincos_processor * scproc, position_t xoff, position_t yoff, position_t aoff)
{
	scproc->x_offset = xoff;
	scproc->x_offset = yoff;
	scproc->x_offset = aoff;
}

void axstate_sincos_processor_set_x_offset(struct axstate_sincos_processor * scproc, position_t xoff)
{
	scproc->x_offset = xoff;
}

void axstate_sincos_processor_set_y_offset(struct axstate_sincos_processor * scproc, position_t yoff)
{
	scproc->x_offset = yoff;
}

void axstate_sincos_processor_set_a_offset(struct axstate_sincos_processor * scproc, position_t aoff)
{
	scproc->x_offset = aoff;
}


void axstate_sincos_processor_init(
    struct axstate_sincos_processor * scproc,
    const char* name,
    struct axis_state ** leftside,
    struct axis_state ** rightside,
    position_t radius
)
{
	signal_processor_init(&scproc->proc, name, &axstate_sincos_processor_ops);

	scproc->leftside = leftside;
	scproc->rightside = rightside;

	scproc -> radius = radius;
	scproc -> x_offset = 0;
	scproc -> y_offset = 0;
	scproc -> a_offset = 0;

	scproc -> alpha_to_radian_scale = 1;
}