#include <ralgo/heimer2/signal_processor.h>

#include <string.h>

DLIST_HEAD(signal_processor_list);

void signal_processor_init(struct signal_processor * processor,
	const char * name,
	const struct signal_processor_operations * ops	
) 
{
	processor->ops = ops;
	strncpy(processor->name, name, SIGNAL_PROCESSOR_NAME_MAX_LENGTH);
	dlist_add_tail(&processor->list_lnk, &signal_processor_list);
}


void signal_processor_deinit(struct signal_processor * processor) 
{
	dlist_del(&processor->list_lnk);
}

void signal_processor_serve(struct signal_processor * processor, disctime_t time) 
{
	processor->ops->serve(processor, time);
}
