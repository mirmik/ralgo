#include <ralgo/heimer2/axstate_sincos_processor.h>

void axstate_sincos_processor_serve(struct signal_processor * proc, disctime_t time) 
{
	struct axstate_sincos_processor * scproc = (struct axstate_sincos_processor *) proc;

	
}

void axstate_sincos_processor_feedback(struct signal_processor * proc, disctime_t time) 
{
	struct axstate_sincos_processor * scproc = (struct axstate_sincos_processor *) proc;
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

void axstate_sincos_processor_init(
	struct axstate_sincos_processor * scproc, 
	const char* name,
	struct axis_state ** leftside,
	struct axis_state ** rightside,
	position_t radius,
	position_t x_offset,
	position_t y_offset
) 
{
	signal_processor_init(&scproc->proc, name, &axstate_sincos_processor_ops);

	scproc->leftside = leftside;
	scproc->rightside = rightside;

	scproc -> radius = radius;
	scproc -> x_offset = x_offset;
	scproc -> y_offset = y_offset;
}