#include <ralgo/heimer/command_center.h>
#include <ralgo/heimer/protect.h>

heimer::command_center_cls<float,float> heimer::command_center;

int heimer::axcmd(int argc, char** argv)
{
	return command_center.axcmd(argc, argv);
}

int heimer::igcmd(int argc, char** argv)
{
	return command_center.igcmd(argc, argv);
}

int heimer::ctrcmd(int argc, char** argv)
{
	return command_center.ctrcmd(argc, argv);
}

int feed(int argc, char** argv)
{
	return heimer::command_center.feed(argc, argv);
}

igris::console_command heimer::command_center_cmdtable[] = { 
	igris::console_command{"axcmd", axcmd}, 
	igris::console_command{"igcmd", igcmd},
	igris::console_command{"ctrcmd", ctrcmd},
	igris::console_command{"ax", axcmd}, 
	igris::console_command{"ig", igcmd},  
	igris::console_command{"ctr", ctrcmd},
	igris::console_command{"feed", feed},
	igris::console_command{"setprotect", set_global_protection_command} 
};