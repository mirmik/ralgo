#include "device.h"
#include <stdio.h>
#include <string.h>

dlist_head virtdevs::vitrdev_list = DLIST_HEAD_INIT(virtdevs::vitrdev_list);


int virtdevs::cli_utility(int argc, char** argv, char* ret, int retmax) 
{
	if (argc == 1) 
	{
		sprintf(ret, "need subfunc");
		return 0;	
	}

	if (strcmp(argv[1], "list")==0) 
	{
		virtdevs::device * dev;
		dlist_for_each_entry(dev, &virtdevs::vitrdev_list, vitrdev_list_lnk) 
		{
			int len = sprintf(ret, "%s\n", dev->name());
			ret += len;
		}
	}	

	else 
	{
		sprintf(ret, "unresolved subfunc");
		return 0;	

	}

	return 0;
}