#ifndef RALGO_command_H
#define RALGO_command_H

#include <igris/compiler.h>

#include <ralgo/heimer/signal.h>
#include <ralgo/heimer/signal_processor.h>

namespace heimer
{
	int command_signal_processors(int argc, char ** argv, char * output, int maxsize);
	int command_signals(int argc, char ** argv, char * output, int maxsize);
	int command(int argc, char ** argv, char * output, int maxsize);

	int command_exec_safe(const char * str, char * output, int maxsize);

	void heimer_reinit();

	void heimer_system_init();

	int command_help(int argc, char ** argv, char * output, int maxsize);

	int signal_processors_command_new(int argc, char ** argv, char * output,int maxsize);
	int signal_head_command_new(int argc, char ** argv, char * output, int maxsize);
}

#endif