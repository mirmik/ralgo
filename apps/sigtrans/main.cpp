#include <string>
#include <iostream>

#include <igris/getopt/cliopts.h>
#include <ralgo/heimer/command.h>

int DEBUG = 0;


int main(int argc, char ** argv) 
{
	igris::cliopts cli;
	cli.add_option("debug", 'd');
	cli.parse(argc, argv);

	DEBUG = cli.get_option("debug");

	while(1) 
	{
		std::string line;
		std::getline(std::cin, line);
		heimer::command(line.c_str(), line.size());
	}
}