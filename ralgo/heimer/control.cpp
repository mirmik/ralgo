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

	if (is_controlled())
		return HEIM_ERR_IS_BUSY;

	if (!(flags & HEIM_IS_ACTIVE))
		return 0;

	if ((sts = on_deactivate())) 
		return sts;

	release_control(); 

	flags &= ~HEIM_IS_ACTIVE;
	return 0;
}

int heimer::control_node::take_control()
{
	int sts = 0;

	control_node* it = nullptr;
	while ((it = iterate(it)))
	{
		if (it->is_active() == false) 
		{
			sts = HEIM_ERR_IS_CONTROL_FAULT;
			break;
		}

		sts = it->on_external_take(this);
		if (sts) break;
		it->flags |= HEIM_IS_CONTROLLED;
	}

	if (sts) release_control();

	return sts;
}

int heimer::control_node::release_control()
{
	int sts = 0;

	control_node* it = nullptr;
	while ((it = iterate(it)))
	{
		if (it->is_active()) 
		{
			sts = it->on_external_release(this);
			it->flags &= ~HEIM_IS_CONTROLLED;
			(void) sts;
		}
	}

	return 0;
}
