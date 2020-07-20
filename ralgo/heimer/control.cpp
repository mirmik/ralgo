#include <ralgo/heimer/control.h>

int heimer::control_node::activate()
{
	int sts;

	if (flags & HEIM_IS_ACTIVE)
		return 0;

	if (sts = take_control())
		return sts;

	if (activate && (sts = node->on_activate()))
	{
		release_control();
		return sts;
	}

	node->flags |= HEIM_IS_ACTIVE;
	return 0;
}