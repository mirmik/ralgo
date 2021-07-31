#include <ralgo/heimer/axstate_linear_processor.h>
#include <ralgo/clinalg/matops.h>
#include <ralgo/heimer/sigtypes.h>

#include <ralgo/linalg/matops.h>
#include <ralgo/log.h>


using namespace heimer;

int axstate_linear_processor::serve(disctime_t time)
{
	(void) time;

	for (int i = 0; i < leftdim(); ++i)
	{
		position_t accpos = 0;
		velocity_t accvel = 0;
		for (int j = 0; j < dim(); ++j)
		{
			accpos += *(matrix + i * dim() + j) * rightax(j)->ctrpos;
			accvel += *(matrix + i * dim() + j) * rightax(j)->ctrvel;
		}

		leftax(i)->ctrpos = accpos;
		leftax(i)->ctrvel = accvel;
	}

	return 0;
}

int axstate_linear_processor::feedback(disctime_t time)
{
	(void) time;

	for (int i = 0; i < dim(); ++i)
	{
		position_t accpos = 0;
		velocity_t accvel = 0;
		for (int j = 0; j < dim(); ++j)
		{
			accpos += *(invert_matrix + i * dim() + j) * leftax(j)->feedpos;
			accvel += *(invert_matrix + i * dim() + j) * leftax(j)->feedvel;
		}

		rightax(i)->feedpos = accpos;
		rightax(i)->feedvel = accvel;
	}

	return 0;
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
int matrix(axstate_linear_processor * axctr, int argc, char ** argv, char * output, int outmax)
{
	if (argc != axctr->dim() * axctr->dim())
	{
		snprintf(output, outmax, "Wrong matrix size");
		return -1;
	}

	for (int i = 0; i < argc; ++i)
	{
		axctr->matrix[i] = atof(argv[i]);
	}

	axctr->evaluate_invertion();

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
		snprintf(buf, bufsize, "%s,", axctr->leftax(i)->name);
		strncat(output, buf, outmax);
	}
	snprintf(buf, bufsize, "%s\r\n", axctr->leftax(axctr->dim() - 1)->name);
	strncat(output, buf, outmax);

	memset(buf, 0, bufsize);
	snprintf(buf, bufsize, "rsignals: ");
	strncat(output, buf, outmax);

	memset(buf, 0, bufsize);
	for (int i = 0; i < axctr->dim() - 1; ++i)
	{
		snprintf(buf, bufsize, "%s,", axctr->rightax(i)->name);
		strncat(output, buf, outmax);
	}
	snprintf(buf, bufsize, "%s\r\n", axctr->rightax(axctr->dim() - 1)->name);
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

	if (strcmp("matrix", argv[0]) == 0)
		status = ::matrix(this, argc - 1, argv + 1, output, outmax);

	if (strcmp("info", argv[0]) == 0)
		status = info(this, argc - 1, argv + 1, output, outmax);

	return status;
}

void axstate_linear_processor::deinit()
{
	for (int i = 0; i < dim(); ++i)
	{
		leftax(i) -> deattach_possible_controller(this);
		rightax(i) -> deattach_listener(this);
	}
}

void axstate_linear_processor::evaluate_invertion()
{
	matops_square_inverse_f(
	    matrix, // in
	    dim(),  //size, n==m
	    dim(),  //stride
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

	attach_leftside_table(leftside, _dim);
	attach_rightside_table(rightside, _dim);

	this->matrix = matrix;
	this->invert_matrix = invert_matrix;

	for (int i = 0; i < _dim; ++i)
	{
		this->rightax(i)->listener = this;
	}
}

heimer::axstate_linear_processor::axstate_linear_processor(const char * name, int _dim)
	: axstate_signal_processor(name)
{
	attach_leftside_table(nullptr, _dim);
	attach_rightside_table(nullptr, _dim);
}

int heimer::axstate_linear_processor::dim()
{
	return leftdim();
}

void heimer::axstate_linear_processor::allocate_resources()
{
	this->attach_leftside_table(new axis_state * [dim()], dim());
	this->attach_rightside_table(new axis_state * [dim()], dim());
	this->matrix = new float[dim() * dim()];
	this->invert_matrix = new float[dim() * dim()];

	ralgo::matrix_view_ro<float> A(this->matrix, dim(), dim());
	ralgo::matrix_view_ro<float> B(this->invert_matrix, dim(), dim());

	ralgo::matops::eye(A);
	ralgo::matops::eye(B);
}


void heimer::axstate_linear_processor::on_activate(disctime_t) 
{
	for (int i = 0; i < dim(); ++i)
	{
		rightax(i)->ctrpos = rightax(i)->feedpos; 
		rightax(i)->ctrvel = rightax(i)->feedvel;
	}
}