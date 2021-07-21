#ifndef RALGO_HEIMER_SIGNAL_H
#define RALGO_HEIMER_SIGNAL_H

#include <igris/compiler.h>
#include <igris/datastruct/dlist.h>

#define SIGNAL_NAME_MAX_LENGTH 8

struct signal_head;

struct signal_head_operations 
{
	int (* info)(struct signal_head *, char * output, int maxsize); 
};

struct signal_head
{
	struct dlist_head signal_list_lnk;
	char name[SIGNAL_NAME_MAX_LENGTH];

	const struct signal_head_operations * ops;
	int16_t refs;
};

__BEGIN_DECLS

void signal_head_init(
	struct signal_head * sig, 
	const char * name, 
	const struct signal_head_operations * ops
);

void signal_head_get(struct signal_head * sig);
void signal_head_put(struct signal_head * sig);

int heimer_signals_count();

void signal_head_list_reinit();

struct signal_head * signals_get_by_name(const char * name);
int signal_command_v(struct signal_head * head, int argc, char ** argv, char * output, int maxsize);

__END_DECLS

#endif