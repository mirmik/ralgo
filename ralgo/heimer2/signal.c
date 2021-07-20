#include <ralgo/heimer2/signal.h>
#include <igris/shell/rshell.h>
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

int heimer_signals_count() 
{
	return dlist_size(&signals_list);
}

void signal_head_list_reinit() 
{
	dlist_init(&signals_list);
}

/// new command
int signal_head_command_new(int argc, char ** argv, char * output) 
{

}

static struct rshell_command heimer_signals_commands[] = {
	{ "new", signal_head_command_new, NULL },
	{ NULL, NULL, NULL }
};

int heimer_command_signals(int argc, char ** argv, char * output) 
{
	int ret;
	int sts = rshell_execute_v(argc, argv, heimer_signals_commands, &ret, 
		1, // drop function name 
		output);
	return 0;	
}