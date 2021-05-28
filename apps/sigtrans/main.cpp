#include <ralgo/heimer/command_center_2.h>

heimer::command_center_2 command_center;

int main(int argc, char ** argv) 
{
	heimer::StatusPair sts;

	(void) argc;
	(void) argv;
	command_center.add_signal("x", 2);
	command_center.add_signal("y", 2);
	command_center.add_signal("z", 2);

	//auto ctr = command_center.create_axis_controller("ctr_y", "y");

	sts = command_center.create_signal_repeater("x_y_repeater", "x", "y");
	if ((bool)sts.status) 
	{
		nos::println("x_y_repeater create error", sts.note);
		exit(-1);
	}

	sts = command_center.create_signal_repeater("y_z_repeater", "y", "z");
	if ((bool)sts.status) 
	{
		nos::println("y_z_repeater create error", sts.note);
		exit(-1);
	}

	command_center.info();
	command_center.sort();
	command_center.info();

	for (int i = 0; i < 10; ++i) 
	{
		command_center.onestep();
	}
}