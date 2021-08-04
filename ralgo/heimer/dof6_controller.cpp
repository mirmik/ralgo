#include <ralgo/heimer/dof6_controller.h>

using namespace heimer;

dof6_controller::dof6_controller(const char * name)
	: signal_processor(name)
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
	return 0;
}

void dof6_controller::deinit()
{

}