#include <ralgo/heimer/command.h>

#include <igris/util/numconvert.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

#include <igris/datastruct/argvc.h>
#include <igris/shell/rshell.h>

#include <igris/dprint.h>

#include <ralgo/heimer/axis_state.h>
#include <ralgo/heimer/axisctr.h>

using namespace heimer;

int heimer::command_help(int, char **, char * output, int maxsize)
{
	snprintf(output, maxsize,
	        "Command list:\r\n"
	        "signal - signal api utility\r\n"
	       );

	return 0;
}

int heimer::command_exec_safe(const char * str, char * output, int maxsize)
{
	char copydata[48];
	char * argv[10];
	int    argc;

	strncpy(copydata, str, 48);

	argc = argvc_internal_split(copydata, argv, 10);
	return command(argc, argv, output, maxsize);
}

void heimer::heimer_system_init()
{
	signal_head_list_reinit();
	signal_processors_list_reinit();
}

/// new command
int heimer::signal_processors_command_new(int, char ** argv, char *, int)
{
	if (strcmp(argv[0], "axisctr") == 0)
	{
		int dim = atoi32(argv[2], 10, NULL);
		create_axis_controller(argv[1], dim);	
		return 0;
	}	

	return -1;
}

static struct rshell_command heimer_signal_processors_commands[] =
{
	//{ "new", signal_processors_command_new, NULL },
	{ NULL, NULL, NULL }
};

int heimer::command_signal_processors(int argc, char ** argv, char * output, int maxsize)
{
	int ret;
	int sts = rshell_execute_v(argc, argv, heimer_signal_processors_commands, &ret,
	                           1, // drop function name
	                           output, maxsize);
	return sts ? sts : ret;
}

/// new command
int heimer::signal_head_command_new(int, char ** argv, char *, int) 
{
	if (strcmp(argv[0], "axstate") == 0)
	{
		new axis_state(argv[1]); // Сохранение указателя происходит внутри конструктора.
		return 0;
	}

	return -1;
}

int heimer::command_signals(int argc, char ** argv, char * output, int maxsize) 
{
	char * signame = argv[0];
	signal_head * sig = signal_get_by_name(signame);

	if (sig == NULL) 
	{
		snprintf(output, maxsize, "Signal not found.");
		return ENOENT;
	}

	return sig->command_v(argc-1, argv+1, output, maxsize);
}

static struct rshell_command commands[] =
{
	{ "help", command_help, NULL },
	{ "sig", command_signals, NULL },
	{ "ctr", command_signal_processors, NULL },
	{ "ctrnew", signal_processors_command_new, NULL },
	{ "signew", signal_head_command_new, NULL },
	{ NULL, NULL, NULL }
};

int heimer::command(int argc, char ** argv, char * output, int maxsize)
{
	int ret;
	int sts = rshell_execute_v(argc, argv, commands, &ret,
	                           1, // drop submenu name
	                           output, maxsize);

	return sts ? sts : ret;
}

void heimer::heimer_reinit() 
{
	signal_head_list_reinit();
	signal_processors_list_reinit();
}