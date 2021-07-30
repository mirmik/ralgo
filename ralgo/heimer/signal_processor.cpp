#include <ralgo/heimer/signal_processor.h>
#include <ralgo/heimer/signal.h>
#include <igris/shell/rshell.h>
#include <igris/math.h>
#include <string.h>
#include <assert.h>

using namespace heimer;

DLIST_HEAD(heimer::signal_processor_list);

void signal_processor::init(const char * name)
{
	int len = MIN(strlen(name), SIGNAL_PROCESSOR_NAME_MAX_LENGTH);
	memset(_name, 0, SIGNAL_PROCESSOR_NAME_MAX_LENGTH);
	memcpy(_name, name, len);
	dlist_add_tail(&list_lnk, &signal_processor_list);
	this->active = 0;
}

signal_processor::signal_processor(const char * name)
{
	init(name);
}

void signal_processor::deinit()
{
	dlist_del(&list_lnk);
}

int heimer::signal_processors_count()
{
	return dlist_size(&signal_processor_list);
}

void heimer::signal_processors_list_reinit()
{
	dlist_init(&signal_processor_list);
}

int signal_processor::activate()
{
	int success = 1;

	signal_head * iter = NULL;
	while ((iter = iterate_left(iter)))
	{
		int err = iter->activate(this);
		if (err)
		{
			success = 0;
			break;
		}
	}

	if (success)
	{
		active = 1;
		on_activate();
		return 0;
	}

	else
	{
		deactivate();
		return -1;
	}
}

int signal_processor::deactivate()
{
	signal_head * iter;
	int all_right_deactivated = 1;

	iter = NULL;
	while ((iter = iterate_right(iter)))
	{
		if (iter->current_controller)
			all_right_deactivated = 0;
	}

	if (!all_right_deactivated)
		return 0;

	iter = NULL;
	while ((iter = iterate_left(iter)))
	{
		int err = iter->deactivate(this);
		(void) err;
	}
	active = 0;

	return 0;
}

signal_processor * heimer::signal_processor_get_by_name(const char * name)
{
	signal_processor * sig;
	dlist_for_each_entry(sig, &signal_processor_list, list_lnk)
	{
		if (strncmp(sig->name().data(), name, SIGNAL_NAME_MAX_LENGTH) == 0)
			return sig;
	}
	return NULL;
}

igris::buffer signal_processor::name()
{
	return { _name, strnlen(_name, SIGNAL_PROCESSOR_NAME_MAX_LENGTH) };
}

bool signal_processor::is_active()
{
	return active;
}
 
void signal_processor::on_activate() 
{}