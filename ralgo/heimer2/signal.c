#include <ralgo/heimer2/signal.h>
#include <ralgo/heimer2/signal_processor.h>
#include <igris/shell/rshell.h>
#include <string.h>

#include <igris/datastruct/nametbl.h>

DLIST_HEAD(signals_list);

void signal_head_init(
	struct signal_head * sig, 
	const char * name, 
	uint8_t type,
	const struct signal_head_operations * ops)
{
	sig->refs = 0;
	sig->ops = ops;
	sig->type = type;
	strncpy(sig->name, name, SIGNAL_NAME_MAX_LENGTH);
	dlist_add_tail(&sig->signal_list_lnk, &signals_list);
	sig->current_controller = NULL;
	sig->listener = NULL;
}

void signal_head_deinit(struct signal_head * sig) 
{
	dlist_del(&sig->signal_list_lnk);	
}


void signal_head_get(struct signal_head * sig)
{
	++sig->refs;
}

void signal_head_put(struct signal_head * sig)
{
	--sig->refs;
}

int heimer_signals_count()
{
	return dlist_size(&signals_list);
}

void signal_head_list_reinit()
{
	dlist_init(&signals_list);
}

struct signal_head * signal_get_by_name(const char * name)
{
	struct signal_head * sig;
	dlist_for_each_entry(sig, &signals_list, signal_list_lnk)
	{
		if (strncmp(name, sig->name, SIGNAL_NAME_MAX_LENGTH) == 0)
			return sig;
	}
	return NULL;
}

int signal_command_v(struct signal_head * head, int argc, char ** argv, char * output, int maxsize)
{	
	char * opsname = argv[0];

	if (strcmp(opsname, "info") == 0) 
		return head->ops->info(head, output, maxsize);

	return -1; 
}


int signal_head_activate(struct signal_head * sig, struct signal_processor * proc) 
{
//	if (!sig->listener)
//		return -1;
	
	if (sig->listener && signal_processor_activate(sig->listener))
		return -1;

	sig->current_controller = proc;
	sig->active = 1;
	return 0;
}

int signal_head_deactivate(struct signal_head * sig, struct signal_processor * proc) 
{
	if (proc != sig->current_controller)
		return -1;

	sig->current_controller = NULL;
	sig->active = -1;

	if (sig->listener) 
	{
		return signal_processor_deactivate(sig->listener);
	} 

	return 0;
}