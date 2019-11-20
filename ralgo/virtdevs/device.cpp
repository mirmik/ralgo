#include "device.h"
#include <stdio.h>

dlist_head virtdevs::vitrdev_list = DLIST_HEAD_INIT(virtdevs::vitrdev_list);


int virtdevs::cli_utility(int argc, char** argv, char* ret, int retmax) 
{
	sprintf(ret, "hello %s", "mirmik");
	return 0;
}