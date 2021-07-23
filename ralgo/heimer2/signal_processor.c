#include <ralgo/heimer2/signal_processor.h>
#include <ralgo/heimer2/signal.h>
#include <igris/shell/rshell.h>
#include <string.h>
#include <assert.h>

DLIST_HEAD(signal_processor_list);

void signal_processor_init(struct signal_processor * processor,
                           const char * name,
                           const struct signal_processor_operations * ops
                          )
{
	processor->ops = ops;
	strncpy(processor->name, name, SIGNAL_PROCESSOR_NAME_MAX_LENGTH);
	dlist_add_tail(&processor->list_lnk, &signal_processor_list);
	processor->active = 0;

	assert(ops->serve);
	assert(ops->feedback);
	assert(ops->command);
	assert(ops->deinit);
	assert(ops->iterate_left);
}


void signal_processor_deinit(struct signal_processor * processor)
{
	dlist_del(&processor->list_lnk);
}

void signal_processor_serve(struct signal_processor * processor, disctime_t time)
{
	processor->ops->serve(processor, time);
}

void signal_processor_feedback(struct signal_processor * processor, disctime_t time)
{
	processor->ops->feedback(processor, time);
}

int heimer_signal_processors_count()
{
	return dlist_size(&signal_processor_list);
}

void signal_processors_list_reinit()
{
	dlist_init(&signal_processor_list);
}

int signal_processor_activate(struct signal_processor * processor)
{
	int success = 1;

	struct signal_head * iter = NULL;
	while (iter = processor->ops->iterate_left(processor, iter))
	{
		int err = signal_head_activate(iter, processor);
		if (err)
		{
			success = 0;
			break;
		}
	}

	if (success)
	{
		processor->active = 1;
		return 0;
	}

	else
	{
		signal_processor_deactivate(processor);
		return -1;
	}
}

int signal_processor_deactivate(struct signal_processor * processor)
{
	struct signal_head * iter = NULL;
	while (iter = processor->ops->iterate_left(processor, iter))
	{
		int err = signal_head_deactivate(iter, processor);
		(void) err;
	}
	processor->active = 0;

	return 0;
}