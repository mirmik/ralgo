#include <ralgo/heimer/axstate_linear_processor.h>
#include <ralgo/clinalg/matops.h>

using namespace heimer;

int axstate_linear_processor::serve(disctime_t time)
{
	(void) time;

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

	return 0;
}

int axstate_linear_processor::feedback(disctime_t time)
{
	(void) time;

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
	
	return 0;
}

int  axstate_linear_processor::command(int argc, char ** argv, char * output, int outmax)
{
	(void) argc;
	(void) argv;
	(void) output;
	(void) outmax;

	return 0;
}

void axstate_linear_processor::deinit()
{}

signal_head * axstate_linear_processor::iterate_left(signal_head * iter)
{
	if (iter == NULL)
		return *leftside;

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

signal_head * axstate_linear_processor::iterate_right(signal_head * iter)
{
	if (iter == NULL)
		return *rightside;

	struct axis_state ** it = rightside;
	for (; it != rightside + dim - 1  ; ++it)
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

	for (int i = 0; i < dim; ++i) 
	{
		this->rightside[i]->listener = this;
	}
}

heimer::axstate_linear_processor::axstate_linear_processor(const char * name, int dim) 
	: signal_processor(name)
{
	this->dim = dim;
}