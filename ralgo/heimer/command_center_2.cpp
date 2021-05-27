#include <ralgo/heimer/command_center_2.h>


heimer::command_record __comands[] = {
	{ "addsignal", &heimer::command_center_2::add_signal_command }
};

heimer::command_record * heimer::command_center_2::_comands = __comands;