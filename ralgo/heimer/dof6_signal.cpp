#include <ralgo/heimer/dof6_signal.h>
#include <ralgo/heimer/sigtypes.h>

heimer::dof6_signal::dof6_signal(const char * name)
	: heimer::signal_head(name, SIGNAL_TYPE_DOF6)
{

}

void heimer::dof6_signal::init(const char * name) 
{
	signal_head::init(name, SIGNAL_TYPE_DOF6);
}

int heimer::dof6_signal::info(char *, int ) 
{
	return 0;
}