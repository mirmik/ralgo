#include <ralgo/heimer/axstate_pose3_chain_processor.h>
#include <ralgo/heimer/sigtypes.h>

using namespace heimer;

axstate_chain3_processor::axstate_chain3_processor(const char * name, int leftdim)
	: signal_processor(name, leftdim, 1)
{}

ralgo::pose3<position_t> axstate_chain3_processor::evaluate_target_position()
{
	return {};
}

ralgo::screw3<position_t> axstate_chain3_processor::evaluate_target_velocity()
{
	return {};
}

ralgo::screw3<position_t> axstate_chain3_processor::evaluate_position_error()
{
	return {};
}

int axstate_chain3_processor::serve(disctime_t)
{
	return 0;
}

static inline
int info(axstate_chain3_processor * axctr, int, char **, char * output, int outmax)
{
	snprintf(output, outmax, "is_active: %d\r\n", axctr->is_active());
	return 0;
}

static inline
int setconstant(axstate_chain3_processor * axctr, int argc, char ** argv, char * output, int outmax)
{
	if (argc != 7)
	{
		snprintf(output, outmax, "Need seven arguments");
		return -1;
	}

	axctr->set_constant(
	    atoi(argv[0]),
	    atof(argv[1]),
	    atof(argv[2]),
	    atof(argv[3]),
	    atof(argv[4]),
	    atof(argv[5]),
	    atof(argv[6])
	);

	return 0;
}

static inline
int setsensivity(axstate_chain3_processor * axctr, int argc, char ** argv, char * output, int outmax)
{
	if (argc != 7)
	{
		snprintf(output, outmax, "Need seven arguments");
		return -1;
	}

	axctr->set_sensivity(
	    atoi(argv[0]),
	    atof(argv[1]),
	    atof(argv[2]),
	    atof(argv[3]),
	    atof(argv[4]),
	    atof(argv[5]),
	    atof(argv[6])
	);

	return 0;
}

int axstate_chain3_processor::command(int argc, char ** argv, char * output, int outmax)
{
	int status = ENOENT;

	if (strcmp("info", argv[0]) == 0)
		status = ::info(this, argc - 1, argv + 1, output, outmax);

	if (strcmp("setconstant", argv[0]) == 0)
		status = ::setconstant(this, argc - 1, argv + 1, output, outmax);

	if (strcmp("setsensivity", argv[0]) == 0)
		status = ::setsensivity(this, argc - 1, argv + 1, output, outmax);

	if (status != ENOENT)
		return status;

	return signal_processor::command(argc, argv, output, outmax);
}

void axstate_chain3_processor::deinit()
{
	release_signals();

	if (settings && is_dynamic_resources())
		delete[] settings;

	if (temporary && is_dynamic_resources())
		delete[] temporary;
}

void axstate_chain3_processor::on_activate(disctime_t)
{

}

void axstate_chain3_processor::evaluate_error()
{

}

void axstate_chain3_processor::evaluate_output_sensivities(ralgo::screw3<double> *)
{

}

void axstate_chain3_processor::backpack(ralgo::screw3<double> *)
{

}

int axstate_chain3_processor::feedback(disctime_t)
{
	ralgo::pose3<position_t> pose;
	for (int i = 0; i < leftdim(); ++i)
	{
		pose = pose * settings[i].constant_transform;
		temporary[i].leftmatrix = pose;
		pose = pose * ralgo::pose3<position_t>::from_screw(settings[i].local_sensivity * leftax(i)->feedpos);
	}
	feedback_position = pose * last_constant_transform;

	pose = last_constant_transform;
	for (int i = leftdim() - 1; i >= 0 ; --i)
	{
		pose = ralgo::pose3<position_t>::from_screw(settings[i].local_sensivity * leftax(i)->feedpos) * pose;
		temporary[i].rightmatrix = pose;
		pose = settings[i].constant_transform * pose;
	}

	ralgo::screw3<velocity_t> velscr = {};
	for (int i = 0; i < leftdim(); ++i)
	{
		velscr +=
		    settings[i].local_sensivity.kinematic_carry(temporary[i].leftmatrix)
		    * leftax(i)->feedvel;
	}

	feedback_output();
	return 0;
}

void axstate_chain3_processor::allocate_resources()
{
	int dim = leftdim();
	set_dynamic_resources_flag(true);

	settings = new axstate_pose3_chain_settings[dim];
	temporary = new axstate_pose3_chain_temporary[dim];
}

axis_state * axstate_chain3_processor::leftax(int i)
{
	return settings[i].controlled;
}

void axstate_chain3_processor::set_constant(int i, float a, float b, float c, float d, float e, float f)
{
	ralgo::pose3<double> * constant;

	if (i == leftdim() - 1)
	{
		constant = &last_constant_transform;
	}

	else
	{
		constant = &settings[i].constant_transform;
	}

	*constant =
	    ralgo::pose3<double>::translation({a, b, c}) *
	    ralgo::pose3<double>::euler_rotation({d, e, f});
}


void axstate_chain3_processor::set_sensivity(int i, float a, float b, float c, float d, float e, float f)
{
	ralgo::screw3<double> * sensivity;
	sensivity = &settings[i].local_sensivity;

	*sensivity = {{a, b, c}, {d, e, f}};
}