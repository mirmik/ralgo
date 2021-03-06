#include <ralgo/heimer2/command.h>

#include <string.h>
#include <stdio.h>
#include <errno.h>

#include <igris/datastruct/argvc.h>
#include <igris/shell/rshell.h>

#include <igris/dprint.h>

#include <ralgo/heimer2/axis_state.h>
#include <ralgo/heimer2/axisctr.h>



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
		create_axis_controller(argv[1]);	
	}	
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
	}
}

int heimer_command_signals(int argc, char ** argv, char * output, int maxsize) 
{
	char * signame = argv[0];
	struct signal_head * sig = signals_get_by_name(signame);

	if (sig == NULL) 
	{
		snprintf(output, maxsize, "Signal not found.");
		return ENOENT;
	}

	return signal_command_v(sig, argc-1, argv+1, output, maxsize);
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
