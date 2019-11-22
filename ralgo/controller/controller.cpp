#include "controller.h"

void ralgo::controller_executor::serve() 
{
	ralgo::controller ctr = ctrs.first();	
	ctrs.scroll_front();
	ctr.serve();
}

void ralgo::controller::serve() 
{
	int ret = exec();
	
	if (ret == RALGO_CONTROLLER_FINISH) 
	{
		dlist_del_init(&controller_executor_lnk);
	}
}