#include <ralgo/heimer2/command.h>
#include <string.h>
#include <stdio.h>

#include <igris/datastruct/argvc.h>
#include <igris/shell/rshell.h>

#include <igris/dprint.h>

int heimer_command_help(int argc, char ** argv, char * output) 
{
		sprintf(output, 
			"Command list:\r\n"
			"signal - signal api utility\r\n"
		);

	return 0;	
}

static struct rshell_command heimer_commands[] = {
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
