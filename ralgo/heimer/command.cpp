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



int heimer_command_help(int argc, char ** argv, char * output, int maxsize)
{
	snprintf(output, maxsize,
	        "Command list:\r\n"
	        "signal - signal api utility\r\n"
	       );

	return 0;
}

int heimer_command_exec_safe(const char * str, char * output, int maxsize)
{
	char copydata[48];
	char * argv[10];
	int    argc;

	strncpy(copydata, str, 48);

	argc = argvc_internal_split(copydata, argv, 10);
	return heimer_command(argc, argv, output, maxsize);
}

void heimer_system_init()
{
	signal_head_list_reinit();
	signal_processors_list_reinit();
}

/// new command
int signal_processors_command_new(int argc, char ** argv, char * output,int maxsize)
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

int heimer_command_signal_processors(int argc, char ** argv, char * output, int maxsize)
{
	int ret;
	int sts = rshell_execute_v(argc, argv, heimer_signal_processors_commands, &ret,
	                           1, // drop function name
	                           output, maxsize);
	return 0;
}

/// new command
int signal_head_command_new(int argc, char ** argv, char * output, int maxsize) 
{
	if (strcmp(argv[0], "axstate") == 0)
	{
		create_axis_state(argv[1]);	
		return 0;
	}

	return -1;
}

int heimer_command_signals(int argc, char ** argv, char * output, int maxsize) 
{
	char * signame = argv[0];
	struct signal_head * sig = signal_get_by_name(signame);

	if (sig == NULL) 
	{
		snprintf(output, maxsize, "Signal not found.");
		return ENOENT;
	}

	return sig->command_v(argc-1, argv+1, output, maxsize);
}

static struct rshell_command heimer_commands[] =
{
	{ "help", heimer_command_help, NULL },
	{ "sig", heimer_command_signals, NULL },
	{ "ctr", heimer_command_signal_processors, NULL },
	{ "ctrnew", signal_processors_command_new, NULL },
	{ "signew", signal_head_command_new, NULL },
	{ NULL, NULL, NULL }
};

int heimer_command(int argc, char ** argv, char * output, int maxsize)
{
	int ret;
	int sts = rshell_execute_v(argc, argv, heimer_commands, &ret,
	                           1, // drop submenu name
	                           output, maxsize);

	return sts ? sts : ret;
}

void heimer_reinit() 
{
	signal_head_list_reinit();
	signal_processors_list_reinit();
}