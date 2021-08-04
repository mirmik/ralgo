#include <ralgo/heimer/dof6_controller.h>
#include <ralgo/heimer/sigtypes.h>

using namespace heimer;

dof6_controller::dof6_controller(const char * name)
	: signal_processor(name, 1, 0)
{}

signal_head * dof6_controller::iterate_left(signal_head * iter)
{
	return iter == nullptr ? controlled : nullptr;
}

signal_head * dof6_controller::iterate_right(signal_head *)
{
	return nullptr;
}

void dof6_controller::set_controlled(dof6_signal * controlled)
{
	this->controlled = controlled;
}

int dof6_controller::feedback(disctime_t time) 
{
	return 0;
}

int dof6_controller::serve(disctime_t time) 
{
	return 0;
}

int dof6_controller::command(int argc, char ** argv, char * output, int outmax) 
{
	return signal_processor::command(argc, argv, output, outmax);
}

void dof6_controller::deinit()
{

}

int dof6_controller::leftsigtype(int i) { return SIGNAL_TYPE_DOF6; }
signal_head * dof6_controller::leftsig(int i) { return controlled; }
void dof6_controller::set_leftsig(int i, signal_head * sig) { controlled = static_cast<dof6_signal*>(sig); }
