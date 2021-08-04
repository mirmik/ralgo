#include <ralgo/heimer/axstate_pose3_chain_processor.h>
#include <ralgo/heimer/sigtypes.h>

using namespace heimer;

axstate_pose3_chain_processor::axstate_pose3_chain_processor(const char * name, int leftdim)
	: signal_processor(name, leftdim, 1)
{}

ralgo::pose3<position_t> axstate_pose3_chain_processor::evaluate_target_position()
{
	return {};
}

ralgo::screw3<position_t> axstate_pose3_chain_processor::evaluate_target_velocity()
{
	return {};
}

ralgo::screw3<position_t> axstate_pose3_chain_processor::evaluate_position_error()
{
	return {};
}

int axstate_pose3_chain_processor::feedback(disctime_t time)
{
	return 0;
}

int axstate_pose3_chain_processor::serve(disctime_t time)
{
	return 0;
}

static inline
int bindleft(axstate_pose3_chain_processor * axctr, int argc, char ** argv, char * output, int outmax)
{
	if (argc != axctr->leftdim())
	{
		snprintf(output, outmax, "Can't bind %d symbols for %d _dim axisctr", argc, axctr->leftdim());
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
int info(axstate_pose3_chain_processor * axctr, int, char **, char * output, int outmax)
{
	snprintf(output, outmax, "is_active: %d\r\n", axctr->is_active());
	return 0;
}

static inline
int setconstant(axstate_pose3_chain_processor * axctr, int argc, char ** argv, char * output, int outmax)
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
int setsensivity(axstate_pose3_chain_processor * axctr, int argc, char ** argv, char * output, int outmax)
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

int axstate_pose3_chain_processor::command(int argc, char ** argv, char * output, int outmax)
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

void axstate_pose3_chain_processor::deinit()
{
	release_signals();

	if (settings && is_dynamic_resources())
		delete[] settings;

	if (temporary && is_dynamic_resources())
		delete[] temporary;
}

void axstate_pose3_chain_processor::on_activate(disctime_t)
{

}

void axstate_pose3_chain_processor::evaluate_error()
{

}

void axstate_pose3_chain_processor::evaluate_output_sensivities(ralgo::screw3<double> * sensivities)
{

}

void axstate_pose3_chain_processor::backpack(ralgo::screw3<double> * sensivities)
{

}

ralgo::pose3<position_t> axstate_pose3_chain_processor::evaluate_feedback_position()
{
	ralgo::pose3<position_t> pose;
	for (int i = 0; i < leftdim(); ++i)
	{
		pose *= settings[i].constant_transform;
		pose *= ralgo::pose3<position_t>::from_screw(settings[i].local_sensivity * leftax(i)->feedpos);
	}
	pose *= last_constant_transform;
	return pose;
}

void axstate_pose3_chain_processor::allocate_resources()
{
	int dim = leftdim();
	set_dynamic_resources_flag(true);

	settings = new axstate_pose3_chain_settings[dim];
	temporary = new axstate_pose3_chain_temporary[dim];
}

axis_state * axstate_pose3_chain_processor::leftax(int i)
{
	return settings[i].controlled;
}

void axstate_pose3_chain_processor::set_leftside(axis_state ** arr)
{
	for (int i = 0; i < leftdim(); ++i)
	{
		settings[i].controlled = arr[i];
	}
}

void axstate_pose3_chain_processor::set_constant(int i, float a, float b, float c, float d, float e, float f)
{
	ralgo::pose3<double> * constant;

	if (i == 0)
	{
		constant = &last_constant_transform;
	}

	else
	{
		constant = &settings[i - 1].constant_transform;
	}

	*constant =
	    ralgo::pose3<double>::translation({a, b, c});
	ralgo::pose3<double>::euler_rotation({d, e, f});
}


void axstate_pose3_chain_processor::set_sensivity(int i, float a, float b, float c, float d, float e, float f)
{
	ralgo::screw3<double> * sensivity;
	sensivity = &settings[i].local_sensivity;

	*sensivity = {{a, b, c}, {d, e, f}};
}

int axstate_pose3_chain_processor::leftsigtype(int i) { return SIGNAL_TYPE_AXIS_STATE; }

int axstate_pose3_chain_processor::rightsigtype(int i) { return SIGNAL_TYPE_DOF6; }

signal_head * axstate_pose3_chain_processor::leftsig(int i) { return settings[i].controlled; }
signal_head * axstate_pose3_chain_processor::rightsig(int i) { return rightside; }

void axstate_pose3_chain_processor::set_leftsig(int i, signal_head * sig) { settings[i].controlled = static_cast<axis_state*>(sig); }
void axstate_pose3_chain_processor::set_rightsig(int i, signal_head * sig) { rightside = static_cast<dof6_signal*>(sig) ; }