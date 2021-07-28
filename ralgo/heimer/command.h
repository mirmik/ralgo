#ifndef RALGO_HEIMER_COMMAND_H
#define RALGO_HEIMER_COMMAND_H

#include <igris/compiler.h>

#include <ralgo/heimer/signal.h>
#include <ralgo/heimer/signal_processor.h>

namespace heimer
{
	int heimer_command_signal_processors(int argc, char ** argv, char * output, int maxsize);
	int heimer_command_signals(int argc, char ** argv, char * output, int maxsize);
	int heimer_command(int argc, char ** argv, char * output, int maxsize);

	int heimer_command_exec_safe(const char * str, char * output, int maxsize);

	void heimer_reinit();

	void heimer_system_init();

	int heimer_command_help(int argc, char ** argv, char * output, int maxsize);

	int signal_processors_command_new(int argc, char ** argv, char * output,int maxsize);
	int signal_head_command_new(int argc, char ** argv, char * output, int maxsize);
}

#endif