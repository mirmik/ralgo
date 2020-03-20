#include <ralgo/heimer/command_center.h>

ralgo::heimer::command_center_cls<float,float> ralgo::heimer::command_center;

int ralgo::heimer::axcmd(int argc, char** argv)
{
	return command_center.axcmd(argc, argv);
}

int ralgo::heimer::igcmd(int argc, char** argv)
{
	return command_center.igcmd(argc, argv);
}

int feed(int argc, char** argv)
{
	return ralgo::heimer::command_center.feed(argc, argv);
}

igris::console_command ralgo::heimer::command_center_cmdtable[] = { 
	igris::console_command{"axcmd", axcmd}, 
	igris::console_command{"igcmd", igcmd},
	igris::console_command{"ax", axcmd}, 
	igris::console_command{"ig", igcmd},  
	igris::console_command{"feed", feed} 
};