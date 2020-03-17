#include <ralgo/heimer/control.h>
#include <nos/print.h>
#include <nos/fprint.h>

dlist_head ralgo::heimer::control_info_node_list = DLIST_HEAD_INIT(ralgo::heimer::control_info_node_list);

int devinfo(int argc, char** argv)
{
	if (argc != 2) { printf("argc != 2\r\n"); return -1; }

	ralgo::heimer::control_info_node * dev;
	dlist_for_each_entry(dev, &ralgo::heimer::control_info_node_list, lnk)
	{
		if (strcmp(dev->mnemo(), argv[1]) == 0)
		{
			dev->print_info();
			return 0;
		}
	}
	printf("unresolved device\r\n");
	return 0;
}

void devlist()
{
	ralgo::heimer::control_info_node * dev;
	dlist_for_each_entry_reverse(dev, &ralgo::heimer::control_info_node_list, lnk)
	{
		nos::fprint("{} ", dev->mnemo());

		if (dev->srv) nos::fprint("srv:(act:{}) ", dev->srv->is_active());
		if (dev->slt) nos::fprint("slt:(ext:{}) ", dev->slt->is_extern_controlled());

		nos::println();
	}
}

igris::console_command ralgo::heimer::info_node_commands[] =
{
	igris::console_command{"devinfo", devinfo},
	igris::console_command{"devlist", devlist}
};
