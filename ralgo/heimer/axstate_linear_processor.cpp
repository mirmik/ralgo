#include <ralgo/heimer/axstate_linear_processor.h>
#include <ralgo/clinalg/matops.h>

void axstate_linear_processor::serve(disctime_t time)
{
	for (int i = 0; i < dim; ++i)
	{
		position_t accpos = 0;
		velocity_t accvel = 0;
		for (int j = 0; j < dim; ++j)
		{
			accpos += *(matrix + i * dim + j) * rightside[j]->ctrpos;
			accvel += *(matrix + i * dim + j) * rightside[j]->ctrvel;
		}

		leftside[i]->ctrpos = accpos;
		leftside[i]->ctrvel = accvel;
	}
}

void axstate_linear_processor::feedback(disctime_t time)
{
	for (int i = 0; i < dim; ++i)
	{
		position_t accpos = 0;
		velocity_t accvel = 0;
		for (int j = 0; j < dim; ++j)
		{
			accpos += *(invert_matrix + i * dim + j) * leftside[j]->feedpos;
			accvel += *(invert_matrix + i * dim + j) * leftside[j]->feedvel;
		}

		rightside[i]->feedpos = accpos;
		rightside[i]->feedvel = accvel;
	}
}

int  axstate_linear_processor::command(int argc, char ** argv, char * output, int outmax)
{
	return 0;
}

void axstate_linear_processor::deinit()
{}

struct signal_head * axstate_linear_processor::iterate_left(struct signal_head * iter)
{
	if (iter == NULL)
		return *leftside;

	if (iter == *(leftside + dim - 1))
		return NULL;

	struct axis_state ** it = leftside;
	for (; it != leftside + dim - 1  ; ++it)
	{
		if (*it == iter)
		{
			it++;
			return *it;
		}
	}

	return NULL;
}

void axstate_linear_processor::evaluate_invertion()
{
	matops_square_inverse_f(
	    matrix, // in
	    dim,  //size, n==m
	    dim,  //stride
	    invert_matrix //out
	);
}

void axstate_linear_processor::init(
    const char * name,
    int dim,
    struct axis_state ** leftside,
    struct axis_state ** rightside,
    float * matrix,
    float * invert_matrix
)
{
	signal_processor::init(name);

	this->dim = dim;
	this->leftside = leftside;
	this->rightside = rightside;
	this->matrix = matrix;
	this->invert_matrix = invert_matrix;
}
