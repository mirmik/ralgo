#include <ralgo/heimer2/signal.h>
#include <string.h>

DLIST_HEAD(signals_list);

void signal_head_init(struct signal_head * sig, const char * name) 
{
	sig->refs = 0;
	strncpy(sig->name, name, SIGNAL_NAME_MAX_LENGTH);
	dlist_add_tail(&sig->signal_list_lnk, &signals_list);
}

void signal_head_get(struct signal_head * sig) 
{
	++sig->refs;
}

void signal_head_put(struct signal_head * sig) 
{
	--sig->refs;
}