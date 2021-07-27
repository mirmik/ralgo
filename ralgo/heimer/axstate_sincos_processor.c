#include <ralgo/heimer/axstate_sincos_processor.h>

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

	position_t a = ar->ctrpos + scproc->a_right_offset;
	float s = heimpos_sin(a * ascale);             // безразмерная величина
	float c = heimpos_cos(a * ascale);             // безразмерная величина
	float avelocity = heimvel_restore(ar->ctrvel) * ascale;         // скорость в радианах

	xl->ctrpos = xr->ctrpos + scproc->x_offset + scproc->radius * c;
	yl->ctrpos = yr->ctrpos + scproc->y_offset + scproc->radius * s;
	al->ctrpos = a + scproc->a_left_offset;

	xl->ctrvel = xr->ctrvel - scproc->radius * s * avelocity; 
	yl->ctrvel = yr->ctrvel + scproc->radius * c * avelocity;
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

	position_t a = al->feedpos - scproc->a_left_offset;
	float s = heimpos_sin(a * ascale);             // безразмерная величина
	float c = heimpos_cos(a * ascale);             // безразмерная величина
	float avelocity = heimvel_restore(al->feedvel) * ascale;         // скорость в радианах

	xr->feedpos = xl->feedpos - scproc->x_offset - scproc->radius * c;
	yr->feedpos = yl->feedpos - scproc->y_offset - scproc->radius * s;
	ar->feedpos = a - scproc->a_right_offset;

	xr->feedvel = xl->feedvel + scproc->radius * s * avelocity; 
	yr->feedvel = yl->feedvel - scproc->radius * c * avelocity;
	ar->feedvel = al->feedvel;
}

int  axstate_sincos_processor_command(struct signal_processor * proc, int argc, char ** argv, char * output, int outmax)
{
	struct axstate_sincos_processor * scproc = (struct axstate_sincos_processor *) proc;
}

void axstate_sincos_processor_deinit(struct signal_processor * proc)
{
	struct axstate_sincos_processor * scproc = (struct axstate_sincos_processor *) proc;
}


struct signal_head * axstate_sincos_processor_iterate_left(struct signal_processor * sigproc, struct signal_head * iter)
{
	struct axstate_sincos_processor * ctr = mcast_out(sigproc, struct axstate_sincos_processor, proc);

	if (iter == NULL)
		return &(*ctr->leftside)->sig;

	if (iter == &(*(ctr->leftside + 2))->sig) 
		return NULL;

	struct axis_state ** it = ctr->leftside;
	for (; it != ctr->leftside + 2; ++it) 
	{
		if (&(*it)->sig == iter) 
		{
			it++;
			return &((*it)->sig);
		}
	}
}

const struct signal_processor_operations axstate_sincos_processor_ops =
{
	.feedback = axstate_sincos_processor_feedback,
	.serve = axstate_sincos_processor_serve,
	.deinit = axstate_sincos_processor_deinit,
	.command = axstate_sincos_processor_command,
	.iterate_left = axstate_sincos_processor_iterate_left
};

void axstate_sincos_processor_set_alpha_scale(
    struct axstate_sincos_processor * scproc,
    float ascale
)
{
	scproc->alpha_to_radian_scale = ascale;
}

void axstate_sincos_processor_set_offset(
	struct axstate_sincos_processor * scproc, 
	position_t xoff, 
	position_t yoff, 
	position_t aloff, 
	position_t aroff)
{
	scproc->x_offset = xoff;
	scproc->y_offset = yoff;
	scproc->a_left_offset = aloff;
	scproc->a_right_offset = aroff;
}

void axstate_sincos_processor_set_x_offset(struct axstate_sincos_processor * scproc, position_t xoff)
{
	scproc->x_offset = xoff;
}

void axstate_sincos_processor_set_y_offset(struct axstate_sincos_processor * scproc, position_t yoff)
{
	scproc->y_offset = yoff;
}

void axstate_sincos_processor_set_a_left_offset(struct axstate_sincos_processor * scproc, position_t aoff)
{
	scproc->a_left_offset = aoff;
}

void axstate_sincos_processor_set_a_right_offset(struct axstate_sincos_processor * scproc, position_t aoff)
{
	scproc->a_right_offset = aoff;
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
	scproc -> a_right_offset = 0;
	scproc -> a_left_offset = 0;

	scproc -> alpha_to_radian_scale = 1;
}