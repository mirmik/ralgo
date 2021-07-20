#ifndef RALGO_HEIMER_SIGNAL_PROCESSOR_H
#define RALGO_HEIMER_SIGNAL_PROCESSOR_H

#include <igris/datastruct/dlist.h>
#include <ralgo/disctime.h>

#define SIGNAL_PROCESSOR_NAME_MAX_LENGTH 8

struct signal_processor;

struct signal_processor_operations 
{	
	void (* feedback)(struct signal_processor *, disctime_t);
	void (* serve)(struct signal_processor *, disctime_t);
};

struct signal_processor 
{
	struct dlist_head list_lnk;
	char name[SIGNAL_PROCESSOR_NAME_MAX_LENGTH];

	const struct signal_processor_operations * ops;
};

__BEGIN_DECLS

void signal_processor_init(
	struct signal_processor * processor,
	const char * name,
	const struct signal_processor_operations * ops	
);

void signal_processor_deinit(
	struct signal_processor * processor
);

void signal_processor_feedback(struct signal_processor * processor, disctime_t time);
void signal_processor_serve(struct signal_processor * processor, disctime_t time);

int heimer_signal_processors_count();

void signal_processors_list_reinit();

int heimer_command_signal_processors(int argc, char ** argv, char * output);

__END_DECLS

#endif