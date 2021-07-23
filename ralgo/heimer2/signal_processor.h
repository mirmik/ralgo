#ifndef RALGO_HEIMER_SIGNAL_PROCESSOR_H
#define RALGO_HEIMER_SIGNAL_PROCESSOR_H

#include <igris/datastruct/dlist.h>
#include <ralgo/disctime.h>

#define SIGNAL_PROCESSOR_NAME_MAX_LENGTH 8

struct signal_processor;

struct signal_processor_operations 
{	
	void (* feedback)(struct signal_processor * proc, disctime_t time);
	void (* serve)(struct signal_processor * proc, disctime_t time);
	int  (* command)(struct signal_processor * proc, int argc, char ** argv, char * output, int outmax);
	void (* deinit)(struct signal_processor *proc);
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

/// feedback отвечает за движение сигнала слева направо.  physical ----> virtual
void signal_processor_feedback(struct signal_processor * processor, disctime_t time);

/// serve отвечает за движение сигнала справа налево. physical <---- virtual
void signal_processor_serve(struct signal_processor * processor, disctime_t time);

int heimer_signal_processors_count();

void signal_processors_list_reinit();

int heimer_command_signal_processors(int argc, char ** argv, char * output,int maxsize);

__END_DECLS

#endif