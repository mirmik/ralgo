#include <ralgo/heimer/axstate_pose3_chain_processor.h>

heimer::axstate_pose3_chain_processor::axstate_pose3_chain_processor(const char * name, int leftdim) 
	: signal_processor(name)
{

}

rabbit::pose3<position_t> heimer::axstate_pose3_chain_processor::evaluate_target_position()
{
	return {};
}

rabbit::screw3<position_t> heimer::axstate_pose3_chain_processor::evaluate_target_velocity()
{
	return {};
}

rabbit::screw3<position_t> heimer::axstate_pose3_chain_processor::evaluate_position_error()
{
	return {};
}

int heimer::axstate_pose3_chain_processor::feedback(disctime_t time)
{
	return 0;
}

int heimer::axstate_pose3_chain_processor::serve(disctime_t time)
{
	return 0;
}

int heimer::axstate_pose3_chain_processor::command(int argc, char ** argv, char * output, int outmax)
{
	return 0;
}

void heimer::axstate_pose3_chain_processor::deinit()
{

}

void heimer::axstate_pose3_chain_processor::on_activate(disctime_t)
{

}

void heimer::axstate_pose3_chain_processor::evaluate_error()
{

}

void heimer::axstate_pose3_chain_processor::evaluate_output_sensivities(rabbit::screw3<double> * sensivities)
{

}

void heimer::axstate_pose3_chain_processor::backpack(rabbit::screw3<double> * sensivities)
{

}

rabbit::pose3<position_t> evaluate_current_position() 
{
	rabbit::pose3<position_t> pose;
	for (int i = 0; i < leftdim(); ++i) 
	{
		pose *= (local_sensivities[i] * leftax(i)->feedpos).to_pose();
		pose *= constant_transform[i+1];
	}
}