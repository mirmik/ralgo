#include <ralgo/heimer2/command.h>
#include <string.h>
#include <stdio.h>

#include <igris/datastruct/argvc.h>
#include <igris/shell/rshell.h>

#include <igris/dprint.h>

#include <ralgo/heimer2/axis_state.h>
#include <ralgo/heimer2/axisctr.h>

int heimer_command_help(int argc, char ** argv, char * output)
{
	sprintf(output,
	        "Command list:\r\n"
	        "signal - signal api utility\r\n"
	       );

	return 0;
}

static struct rshell_command heimer_commands[] =
{
	{ "help", heimer_command_help, NULL },
	{ "sig", heimer_command_signals, NULL },
	{ "ctr", heimer_command_signal_processors, NULL },
	{ NULL, NULL, NULL }
};

int heimer_command(int argc, char ** argv, char * output)
{
	int ret;
	int sts = rshell_execute_v(argc, argv, heimer_commands, &ret,
	                           1, // drop submenu name
	                           output);
	return 0;
}


int heimer_command_exec_safe(const char * str, char * output)
{
	char copydata[48];
	char * argv[10];
	int    argc;

	strcpy(copydata, str);

	argc = argvc_internal_split(copydata, argv, 10);
	return heimer_command(argc, argv, output);
}

void heimer_system_init()
{
	signal_head_list_reinit();
	signal_processors_list_reinit();
}

/// new command
int signal_processors_command_new(int argc, char ** argv, char * output)
{
	if (strcmp(argv[0], "axisctr") == 0)
	{
		create_axis_controller(argv[1]);	
	}	
}

static struct rshell_command heimer_signal_processors_commands[] =
{
	{ "new", signal_processors_command_new, NULL },
	{ NULL, NULL, NULL }
};

int heimer_command_signal_processors(int argc, char ** argv, char * output)
{
	int ret;
	int sts = rshell_execute_v(argc, argv, heimer_signal_processors_commands, &ret,
	                           1, // drop function name
	                           output);
	return 0;
}

/// new command
int signal_head_command_new(int argc, char ** argv, char * output) 
{
	if (strcmp(argv[0], "axstate") == 0)
	{
		create_axis_state(argv[1]);	
	}
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