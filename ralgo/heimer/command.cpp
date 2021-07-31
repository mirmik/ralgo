#include <ralgo/heimer/command.h>

#include <igris/util/numconvert.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

#include <igris/datastruct/argvc.h>
#include <igris/shell/rshell.h>
#include <igris/dprint.h>

#include <ralgo/heimer/axis_state.h>
#include <ralgo/heimer/scalar_signal.h>

#include <ralgo/heimer/axisctr.h>
#include <ralgo/heimer/axstate_linear_processor.h>
#include <ralgo/heimer/axstate_sincos_processor.h>
#include <ralgo/heimer/axis_stub_processor.h>
#include <ralgo/heimer/axstate_pid_processor.h>

using namespace heimer;

static
int help(int, char **, char * output, int maxsize)
{
	snprintf(output, maxsize,
	         "Command list:\r\n"
	         "signal - signal api utility\r\n"
	        );

	return 0;
}

int heimer::command_exec_safe(const char * str, char * output, int maxsize, int * ret)
{
	char copydata[48];
	char * argv[10];
	int    argc;

	strncpy(copydata, str, 48);

	argc = argvc_internal_split(copydata, argv, 10);
	return command(argc, argv, output, maxsize, ret);
}

void heimer::heimer_system_init()
{
	signal_head_list_reinit();
	signal_processors_list_reinit();
}

static
int ctrnew(int argc, char ** argv, char * output, int maxsize)
{
	if (argc < 2)
	{
		snprintf(output, maxsize, "usage: ctrnew TYPE NAME\r\n");
		return -1;
	}

	if (strcmp(argv[0], "axisctr") == 0)
	{
		if (argc < 3)
		{
			snprintf(output, maxsize, "usage: ctrnew axisctr NAME DIM\r\n");
			return -1;
		}

		int dim = atoi32(argv[2], 10, NULL);
		create_axis_controller(argv[1], dim);
		return 0;
	}

	if (strcmp(argv[0], "axlinear") == 0)
	{
		if (argc < 3)
		{
			snprintf(output, maxsize, "usage: ctrnew axlinear NAME DIM\r\n");
			return -1;
		}

		int dim = atoi32(argv[2], 10, NULL);
		auto ptr = new heimer::axstate_linear_processor(argv[1], dim);
		ptr->allocate_resources();
		return 0;
	}

	if (strcmp(argv[0], "axsincos") == 0)
	{
		if (argc < 2)
		{
			snprintf(output, maxsize, "usage: ctrnew axsincos NAME\r\n");
			return -1;
		}

		new heimer::axstate_sincos_processor(argv[1]);
		return 0;
	}

	if (strcmp(argv[0], "axstub") == 0)
	{
		if (argc < 2)
		{
			snprintf(output, maxsize, "usage: ctrnew axstub NAME\r\n");
			return -1;
		}

		new heimer::axis_stub_processor(argv[1]);
		return 0;
	}

	if (strcmp(argv[0], "axpid") == 0)
	{
		if (argc < 2)
		{
			snprintf(output, maxsize, "usage: ctrnew axstub NAME\r\n");
			return -1;
		}

		new heimer::axstate_pid_processor(argv[1]);
		return 0;
	}

	snprintf(output, maxsize, "Unresolved TYPE. Possible types: axisctr, axlinear, axsincos, axstub\r\n");
	return -1;
}

static
int ctr(int argc, char ** argv, char * output, int maxsize)
{
	char * name = argv[0];
	signal_processor * ctr = signal_processor_get_by_name(name);

	if (ctr == NULL)
	{
		snprintf(output, maxsize, "Processor not found.");
		return ENOENT;
	}

	return ctr->command(argc - 1, argv + 1, output, maxsize);
}

static
int signew(int argc, char ** argv, char * output, int maxsize)
{
	if (argc < 2)
	{
		snprintf(output, maxsize, "usage: signew TYPE NAME\r\n");
		return -1;
	}

	if (strcmp(argv[0], "axstate") == 0)
	{
		const char * name = argv[1];
		new axis_state(name); // Сохранение указателя происходит внутри конструктора.
		return 0;
	}

	if (strcmp(argv[0], "scalar") == 0)
	{
		const char * name = argv[1];
		new scalar_signal(name);
		return 0;
	}

	snprintf(output, maxsize, "Unresolved TYPE. Possible types: axstate\r\n");
	return -1;
}

static
int sig(int argc, char ** argv, char * output, int maxsize)
{
	char * signame = argv[0];
	signal_head * sig = signal_get_by_name(signame);

	if (sig == NULL)
	{
		snprintf(output, maxsize, "Signal not found.");
		return ENOENT;
	}

	return sig->command_v(argc - 1, argv + 1, output, maxsize);
}

static
int ctrlist(int, char **, char * output, int maxsize)
{
	char buf[48];

	signal_processor * it;
	dlist_for_each_entry(it, &signal_processor_list, list_lnk)
	{
		snprintf(buf, 48,
		         "%*s\r\n", SIGNAL_PROCESSOR_NAME_MAX_LENGTH, it->name().data());
		strncat(output, buf, maxsize);
	}

	return 0;
}

static
int siglist(int, char **, char * output, int maxsize)
{
	signal_head * it;
	dlist_for_each_entry(it, &signals_list, list_lnk)
	{
		char buf[SIGNAL_PROCESSOR_NAME_MAX_LENGTH + 4];
		snprintf(buf, SIGNAL_PROCESSOR_NAME_MAX_LENGTH + 4,
		         "%*s\r\n", SIGNAL_PROCESSOR_NAME_MAX_LENGTH, it->name);
		strncat(output, buf, maxsize);
	}

	return 0;
}

static struct rshell_command commands[] =
{
	{ "help", help, NULL },
	{ "sig", sig, NULL },
	{ "ctr", ctr, NULL },
	{ "ctrnew", ctrnew, NULL },
	{ "signew", signew, NULL },
	{ "ctrlist", ctrlist, NULL },
	{ "siglist", siglist, NULL },
	{ NULL, NULL, NULL }
};

int heimer::command(int argc, char ** argv, char * output, int maxsize, int * ret)
{
	int sts = rshell_execute_v(argc, argv, commands, ret,
	                           1, // drop submenu name
	                           output, maxsize);

	return sts;
}

void heimer::heimer_reinit()
{
	signal_head_list_reinit();
	signal_processors_list_reinit();
}