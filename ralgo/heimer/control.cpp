#include <ralgo/heimer/control.h>

int heimer::control_node::activate()
{
	int sts;

	if (flags & HEIM_IS_ACTIVE)
		return 0;

	if ((sts = take_control()))
		return sts;

	if ((sts = on_activate()))
	{
		release_control();
		return sts;
	}

	flags |= HEIM_IS_ACTIVE;
	return 0;
}

int heimer::control_node::deactivate() 
{
	int sts;

	if (!(flags & HEIM_IS_ACTIVE))
		return 0;

	if ((sts = on_deactivate())) 
		return sts;

	release_control(); 

	flags &= ~HEIM_IS_ACTIVE;
	return 0;
}
