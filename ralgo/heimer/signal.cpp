#include <ralgo/heimer/signal.h>
#include <ralgo/heimer/signal_processor.h>
#include <igris/shell/rshell.h>
#include <string.h>

#include <igris/datastruct/nametbl.h>

using namespace heimer;

DLIST_HEAD(heimer::signals_list);

void signal_head::init(const char * name, uint8_t type)
{
	refs = 0;
	this->type = type;
	strncpy(this->name, name, SIGNAL_NAME_MAX_LENGTH);
	dlist_add_tail(&signal_list_lnk, &signals_list);
	current_controller = NULL;
	listener = NULL;
}

void signal_head::deinit() 
{
	dlist_del(&signal_list_lnk);	
}


void signal_head::get()
{
	++refs;
}

void signal_head::put()
{
	--refs;
}

int heimer::heimer_signals_count()
{
	return dlist_size(&signals_list);
}

void heimer::signal_head_list_reinit()
{
	dlist_init(&signals_list);
}

signal_head * heimer::signal_get_by_name(const char * name)
{
	signal_head * sig;
	dlist_for_each_entry(sig, &signals_list, signal_list_lnk)
	{
		if (strncmp(sig->name, name, SIGNAL_NAME_MAX_LENGTH) == 0)
			return sig;
	}
	return NULL;
}

int signal_head::command_v(int argc, char ** argv, char * output, int maxsize)
{	
	(void) argc;
	char * opsname = argv[0];

	if (strcmp(opsname, "info") == 0) 
		return info(output, maxsize);

	return ENOENT; 
}


int signal_head::activate(struct signal_processor * proc) 
{
//	if (!listener)
//		return -1;
	
	if (listener && listener->activate())
		return -1;

	current_controller = proc;
	active = 1;
	return 0;
}

int signal_head::deactivate(struct signal_processor * proc) 
{
	if (proc != current_controller)
		return -1;

	current_controller = NULL;
	active = -1;

	if (listener) 
	{
		return listener->deactivate();
	} 

	return 0;
}