#include <string>
#include <iostream>
#include <errno.h>

#include <igris/getopt/cliopts.h>
#include <igris/util/string.h>

#include <ralgo/heimer/command.h>
#include <ralgo/heimer/executor.h>

#include <nos/io/file.h>
#include <nos/fprint.h>
#include <nos/print.h>

#include <crow/gates/udpgate.h>
#include <crow/address.h>
#include <crow/tower.h>

#include <chrono>
#include <thread>
#include <memory>

int DEBUG = 0;

std::unique_ptr<heimer::executor> executor;
std::unique_ptr<std::thread> execute_thread;
int started = 0;
int cancel_token;

crow::udpgate udpgate;
crow::hostaddr crowaddr;

void execute_routine() 
{
	while(1) 
	{
		if (cancel_token) return;
		executor->exec(ralgo::discrete_time());
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

void start_routine() 
{
	if (started) 
	{
		nos::println("It has start early");
		return;
	}
	
	started = 1;
	cancel_token = 0;
	executor.reset(new heimer::executor);
	executor->allocate_order_table(heimer::signal_processors_count());

	heimer::signal_processor * proc;
	dlist_for_each_entry(proc, &heimer::signal_processor_list, list_lnk) 
	{
		executor->append_processor(proc);
	}
	executor->order_sort();

	execute_thread.reset(new std::thread(execute_routine));
}

void stop_routine() 
{
	if (!started)  
	{
		nos::println("Is not started yet");
		return;
	}

	started = 0;
	cancel_token = 1;
	execute_thread->join();
}

void execinfo() 
{
	if (!started) 
	{
		nos::println("Is not started yet");
		return;
	}

	for (int i = 0; i < executor->order_table_size; ++i) 
	{
		heimer::signal_processor * proc = executor->order_table[i];
		nos::println(std::string_view(proc->name().data(), proc->name().size()));
	}
}

void exec(const std::string & line)
{
	int sts;
	char output[256];
	memset(output, 0, 256);

	if (line.size() == 0)
		return;

	if (igris::trim(line) == "start") 
	{
		start_routine();
		return;
	}

	if (igris::trim(line) == "stop") 
	{
		stop_routine();
		return;
	}

	if (igris::trim(line) == "execinfo") 
	{
		execinfo();
		return;
	}

	int ret;
	sts = heimer::command_exec_safe(line.c_str(), output, 256, &ret);

	if (sts == ENOENT)
	{
		nos::println("Command not found.");
	}

	else
	{
		if (DEBUG) 
		{
			nos::fprintln("({}) : ", ret);
		}

		nos::print(output);
	}
}

int main(int argc, char ** argv)
{
	crowaddr = crow::crowker_address();

	igris::cliopts cli;
	cli.add_option("debug", 'd');
	cli.add_string("script", 's', "");
	cli.parse(argc, argv);

	DEBUG = cli.get_option("debug");
	auto script_path = cli.get_string("script").unwrap();

	if (script_path.size())
	{
		std::string str;
		nos::file fl(script_path.c_str(), O_RDONLY);

		while (( str = fl.readline() ).size())
		{
			str = igris::trim(str);

			nos::println("script:", str);
			exec(str);
		}
	}

	crow::start_spin();

	while (1)
	{
		std::string line;
		std::getline(std::cin, line);

		exec(line);
	}

	crow::stop_spin();
}