#include <ralgo/heimer/signal_processor.h>
#include <ralgo/heimer/signal.h>
#include <igris/shell/rshell.h>
#include <string.h>
#include <assert.h>

DLIST_HEAD(signal_processor_list);

void signal_processor::init(const char * name)
{
	strncpy(this->name, name, SIGNAL_PROCESSOR_NAME_MAX_LENGTH);
	dlist_add_tail(&list_lnk, &signal_processor_list);
	this->active = 0;
}


void signal_processor::deinit()
{
	dlist_del(&list_lnk);
}

int heimer_signal_processors_count()
{
	return dlist_size(&signal_processor_list);
}

void signal_processors_list_reinit()
{
	dlist_init(&signal_processor_list);
}

int signal_processor::activate()
{
	int success = 1;

	struct signal_head * iter = NULL;
	while (iter = iterate_left(iter))
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
	struct signal_head * iter = NULL;
	while (iter = iterate_left(iter))
	{
		int err = iter->deactivate(this);
		(void) err;
	}
	active = 0;

	return 0;
}