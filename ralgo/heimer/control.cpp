#include <ralgo/heimer/control.h>

dlist_head heimer::control_node_list =
    DLIST_HEAD_INIT(control_node_list);

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
		it->controller = this;
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
			it->controller = nullptr;
			(void) sts;
		}
	}

	return 0;
}

void heimer::control_node::on_interrupt_common(
    control_node * slave, // источник, переславший сигнал
    control_node * source, // изначальный источник сигнала
    interrupt_args* interrupt)
{
	on_interrupt(slave, source, interrupt);

	if (controller)
		controller->on_interrupt_common(this, source, interrupt);
}

void heimer::control_node::throw_interrupt(interrupt_args* interrupt)
{
	on_interrupt_common(this, this, interrupt);
}