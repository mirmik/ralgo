#include <ralgo/heimer/axstate_linear_processor.h>
#include <ralgo/clinalg/matops.h>
#include <ralgo/heimer/sigtypes.h>

#include <ralgo/linalg/matops.h>
#include <ralgo/log.h>


using namespace heimer;

int axstate_linear_processor::serve(disctime_t time)
{
	(void) time;

	for (int i = 0; i < _dim; ++i)
	{
		position_t accpos = 0;
		velocity_t accvel = 0;
		for (int j = 0; j < _dim; ++j)
		{
			accpos += *(matrix + i * _dim + j) * rightside[j]->ctrpos;
			accvel += *(matrix + i * _dim + j) * rightside[j]->ctrvel;
		}

		leftside[i]->ctrpos = accpos;
		leftside[i]->ctrvel = accvel;
	}

	return 0;
}

int axstate_linear_processor::feedback(disctime_t time)
{
	(void) time;

	for (int i = 0; i < _dim; ++i)
	{
		position_t accpos = 0;
		velocity_t accvel = 0;
		for (int j = 0; j < _dim; ++j)
		{
			accpos += *(invert_matrix + i * _dim + j) * leftside[j]->feedpos;
			accvel += *(invert_matrix + i * _dim + j) * leftside[j]->feedvel;
		}

		rightside[i]->feedpos = accpos;
		rightside[i]->feedvel = accvel;
	}

	return 0;
}

void axstate_linear_processor::set_leftside(axis_state ** arr)
{
	if (!leftside) 
	{
		ralgo::warn("axlinear leftside is not allocated");
		return;
	}

	for (int i = 0; i < _dim; ++i)
		leftside[i] = arr[i];
}

void axstate_linear_processor::set_rightside(axis_state ** arr)
{
	if (!rightside) 
	{
		ralgo::warn("axlinear rightside is not allocated");
		return;
	}

	for (int i = 0; i < _dim; ++i)
		rightside[i] = arr[i];
}

static inline
int bindleft(axstate_linear_processor * axctr, int argc, char ** argv, char * output, int outmax)
{
	if (argc != axctr->dim())
	{
		snprintf(output, outmax, "Can't bind %d symbols for %d _dim axisctr", argc, axctr->dim());
		return -1;
	}

	{
		axis_state * arr[argc];

		for (int i = 0; i < argc; ++i)
		{
			signal_head * sig = signal_get_by_name(argv[i]);

			if (!sig)
			{
				snprintf(output, outmax, "Wrong signal name '%s ' (type 'siglist' for display)", argv[i]);
				return -1;
			}

			if (sig->type != SIGNAL_TYPE_AXIS_STATE)
			{
				snprintf(output, outmax, "Wrong signal type. name:(%s)", sig->name);
				return -1;
			}

			arr[i] = static_cast<axis_state *>(sig);
		}

		axctr->set_leftside(arr);
	}

	return 0;
}

static inline
int bindright(axstate_linear_processor * axctr, int argc, char ** argv, char * output, int outmax)
{
	if (argc != axctr->dim())
	{
		snprintf(output, outmax, "Can't bind %d symbols for %d _dim axisctr", argc, axctr->dim());
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

		axctr->set_rightside(arr);
	}

	return 0;
}


static inline
int info(axstate_linear_processor * axctr, int argc, char ** argv, char * output, int outmax)
{
	(void) argc;
	(void) argv;

	int bufsize = 96;
	char buf[bufsize];

	memset(buf, 0, bufsize);
	snprintf(buf, bufsize, "lsignals: ");
	strncat(output, buf, outmax);

	memset(buf, 0, bufsize);
	for (int i = 0; i < axctr->dim() - 1; ++i)
	{
		snprintf(buf, bufsize, "%s,", axctr->leftside[i]->name);
		strncat(output, buf, outmax);
	}
	snprintf(buf, bufsize, "%s\r\n", axctr->leftside[axctr->dim() - 1]->name);
	strncat(output, buf, outmax);

	memset(buf, 0, bufsize);
	snprintf(buf, bufsize, "rsignals: ");
	strncat(output, buf, outmax);

	memset(buf, 0, bufsize);
	for (int i = 0; i < axctr->dim() - 1; ++i)
	{
		snprintf(buf, bufsize, "%s,", axctr->rightside[i]->name);
		strncat(output, buf, outmax);
	}
	snprintf(buf, bufsize, "%s\r\n", axctr->rightside[axctr->dim() - 1]->name);
	strncat(output, buf, outmax);

	memset(buf, 0, bufsize);
	snprintf(buf, bufsize, "matrix: ");
	strncat(output, buf, outmax);
	for (int i = 0; i < axctr->dim() * axctr->dim(); ++i)
	{
		snprintf(buf, bufsize, "%f ", axctr->matrix[i]);
		strncat(output, buf, outmax);
	}
	snprintf(buf, bufsize, "\r\n");
	strncat(output, buf, outmax);

	memset(buf, 0, bufsize);
	snprintf(buf, bufsize, "invmat: ");
	strncat(output, buf, outmax);
	for (int i = 0; i < axctr->dim() * axctr->dim(); ++i)
	{
		snprintf(buf, bufsize, "%f ", axctr->matrix[i]);
		strncat(output, buf, outmax);
	}
	snprintf(buf, bufsize, "\r\n");
	strncat(output, buf, outmax);

	return 0;
}

int  axstate_linear_processor::command(int argc, char ** argv, char * output, int outmax)
{

	int status = ENOENT;

	if (strcmp("bindleft", argv[0]) == 0)
		status = bindleft(this, argc - 1, argv + 1, output, outmax);

	if (strcmp("bindright", argv[0]) == 0)
		status = bindright(this, argc - 1, argv + 1, output, outmax);

	if (strcmp("info", argv[0]) == 0)
		status = info(this, argc - 1, argv + 1, output, outmax);

	return status;
}

void axstate_linear_processor::deinit()
{}

signal_head * axstate_linear_processor::iterate_left(signal_head * iter)
{
	if (!leftside) 
	{
		ralgo::warn("axlinear leftside is not allocated");
		return nullptr;
	}

	if (iter == NULL)
		return *leftside;

	struct axis_state ** it = leftside;
	for (; it != leftside + _dim - 1  ; ++it)
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
	if (!rightside) 
	{
		ralgo::warn("axlinear leftside is not allocated");
		return nullptr;
	}

	if (iter == NULL)
		return *rightside;

	struct axis_state ** it = rightside;
	for (; it != rightside + _dim - 1  ; ++it)
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
	    _dim,  //size, n==m
	    _dim,  //stride
	    invert_matrix //out
	);
}

void axstate_linear_processor::init(
    const char * name,
    int _dim,
    struct axis_state ** leftside,
    struct axis_state ** rightside,
    float * matrix,
    float * invert_matrix
)
{
	signal_processor::init(name);

	this->_dim = _dim;
	this->leftside = leftside;
	this->rightside = rightside;
	this->matrix = matrix;
	this->invert_matrix = invert_matrix;

	for (int i = 0; i < _dim; ++i)
	{
		this->rightside[i]->listener = this;
	}
}

heimer::axstate_linear_processor::axstate_linear_processor(const char * name, int _dim)
	: signal_processor(name)
{
	this->_dim = _dim;
}

int heimer::axstate_linear_processor::dim()
{
	return _dim;
}

void heimer::axstate_linear_processor::allocate_resources() 
{
	this->leftside = new axis_state * [_dim];
	this->rightside = new axis_state * [_dim];
	this->matrix = new float[_dim*_dim];
	this->invert_matrix = new float[_dim*_dim];

	ralgo::matrix_view_ro<float> A(this->matrix, _dim, _dim);
	ralgo::matrix_view_ro<float> B(this->invert_matrix, _dim, _dim);
	
	ralgo::matops::eye(A);
	ralgo::matops::eye(B);

	for(int i = 0; i < _dim; ++i) 
	{
		leftside[i] = nullptr;
		rightside[i] = nullptr;
	}
}