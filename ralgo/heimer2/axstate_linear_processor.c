#include <ralgo/heimer2/axstate_linear_processor.h>
#include <ralgo/clinalg/matops.h>

void axstate_linear_processor_serve(struct signal_processor * proc, disctime_t time) 
{
	struct axstate_linear_processor * lproc = (struct axstate_linear_processor *) proc;
	int dim = lproc->dim;

	for (int i = 0; i < dim; ++i) 
	{
		int64_t accpos = 0;
		float   accvel = 0;
		for (int j = 0; j < dim; ++j) 
		{
			accpos += *(lproc->matrix + i * dim + j) * lproc->rightside[j]->ctrpos;
			accvel += *(lproc->matrix + i * dim + j) * lproc->rightside[j]->ctrvel;
		}

		lproc->leftside[i]->ctrpos = accpos;
		lproc->leftside[i]->ctrvel = accvel;
	}
}

void axstate_linear_processor_feedback(struct signal_processor * proc, disctime_t time) 
{
	struct axstate_linear_processor * lproc = (struct axstate_linear_processor *) proc;
	int dim = lproc->dim;

	for (int i = 0; i < dim; ++i) 
	{
		int64_t accpos = 0;
		float   accvel = 0;
		for (int j = 0; j < dim; ++j) 
		{
			accpos += *(lproc->invert_matrix + i * dim + j) * lproc->leftside[j]->feedpos;
			accvel += *(lproc->invert_matrix + i * dim + j) * lproc->leftside[j]->feedvel;
		}

		lproc->rightside[i]->feedpos = accpos;
		lproc->rightside[i]->feedvel = accvel;
	}
}

int  axstate_linear_processor_command(struct signal_processor * proc, int argc, char ** argv, char * output, int outmax) 
{
	struct axstate_linear_processor * lproc = (struct axstate_linear_processor *) proc;
}

void axstate_linear_processor_deinit(struct signal_processor * proc) 
{
	struct axstate_linear_processor * lproc = (struct axstate_linear_processor *) proc;	
}

const struct signal_processor_operations axstate_linear_processor_ops =
{
	.feedback = axstate_linear_processor_feedback,
	.serve = axstate_linear_processor_serve,
	.deinit = axstate_linear_processor_deinit,
	.command = axstate_linear_processor_command,
};

void axstate_linear_processor_evaluate_invertion(struct axstate_linear_processor * lproc) 
{
	matops_square_inverse_f(
		lproc->matrix, // in
		lproc->dim,  //size, n==m
		lproc->dim,  //stride
		lproc->invert_matrix //out
	);
}

void axstate_linear_processor_init(
	struct axstate_linear_processor * lproc, 
	const char * name,
	int dim, 
	struct axis_state ** leftside,
	struct axis_state ** rightside,
	float * matrix,
	float * invert_matrix
)
{
	signal_processor_init(&lproc->proc, name, &axstate_linear_processor_ops);

	lproc->dim = dim;
	lproc->leftside = leftside;
	lproc->rightside = rightside;
	lproc->matrix = matrix;
	lproc->invert_matrix = invert_matrix;
}
