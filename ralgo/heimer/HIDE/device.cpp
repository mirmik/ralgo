#include <ralgo/heimer/device.h>

dlist_head ralgo::heimer::device::devices_list = DLIST_HEAD_INIT(devices_list);

int devinfo(int argc, char** argv) 
{
	if (argc != 2) { printf("argc != 2\r\n"); return-1; }

	ralgo::heimer::device * dev;
	dlist_for_each_entry(dev, &ralgo::heimer::device::devices_list, lnk) 
	{
		if(strcmp(dev->name(), argv[1]) == 0) 
		{
			dev->print_info();
			return 0;
		}
	}
	printf("unresolved device\r\n");
	return 0;
}

igris::console_command ralgo::heimer::device_commands[] = 
{
	igris::console_command{"devinfo", devinfo}
};