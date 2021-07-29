#include <string>
#include <iostream>
#include <errno.h>

#include <igris/getopt/cliopts.h>
#include <igris/util/string.h>

#include <ralgo/heimer/command.h>

#include <nos/io/file.h>
#include <nos/fprint.h>
#include <nos/print.h>

int DEBUG = 0;

void exec(const std::string & line)
{
	char output[256];
	memset(output, 0, 256);

	if (line.size() == 0)
		return;

	int ret;
	int sts = heimer::command_exec_safe(line.c_str(), output, 256, &ret);

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

	while (1)
	{
		std::string line;
		std::getline(std::cin, line);

		exec(line);
	}
}