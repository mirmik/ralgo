#include <igris/datastruct/dlist.h>
#include <ralgo/heimer/control.h>
#include <nos/print.h>
#include <nos/fprint.h>

dlist_head ralgo::heimer::control_info_node_list = DLIST_HEAD_INIT(ralgo::heimer::control_info_node_list);

int ralgo::heimer::external_controller::take_control()
{
	int sts = 0;

	external_control_slot* it = nullptr;
	while ((it = iterate(it)))
	{
		sts = it->try_take_external_control(this);
		if (sts) break;
	}

	if (sts) release_control();

	return sts;
}

int ralgo::heimer::external_controller::release_control()
{
	int sts = 0;

	external_control_slot* it = nullptr;
	while ((it = iterate(it)))
	{
		sts = it->try_release_external_control(this);
	}

	(void) sts;
	return 0;
}


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

int devctr(int argc, char** argv)
{
	if (argc < 2) { nos::println("usage: devctr AXNAME CMD [ARGS ...]"); return 0; }

	ralgo::heimer::control_info_node * it;
	dlist_for_each_entry(it, &ralgo::heimer::control_info_node_list, lnk)
	{
		if (strcmp(argv[1], it->mnemo()) == 0) break;
	}
	if (&it->lnk == &ralgo::heimer::control_info_node_list) { nos::println("dev not found"); }

	if (it->srv)
	{
		if (strcmp(argv[2], "activate") == 0)
		{
			int sts = it->srv->activate();
			if (sts) { nos::println("activation fault"); return 0; }
		}

		if (strcmp(argv[2], "deactivate") == 0)
		{
			int sts = it->srv->deactivate();
			if (sts) { nos::println("deactivation fault"); return 0; }
		}
	}

	return 0;
}

igris::console_command ralgo::heimer::info_node_commands[] =
{
	igris::console_command{"devinfo", devinfo},
	igris::console_command{"devlist", devlist},
	igris::console_command{"devctr", devctr}
};
