#include <ralgo/heimer/axis_state.h>
#include <stdlib.h>
#include <stdio.h>

#include <ralgo/heimer/sigtypes.h>

heimer::axis_state::axis_state(const char * name) 
{
	init(name);
}

int heimer::axis_state::info(char * data, int maxsize) 
{
	snprintf(data, maxsize, "(ctrpos:%f,ctrvel:%f,feedpos:%f,feedspd:%f,controller:%016lx)\r\n", 
		ctrpos, ctrvel, feedpos, feedvel, (uintptr_t)current_controller);
	return 0;
}

void heimer::axis_state::init(const char * name) 
{
	signal_head::init(name, SIGNAL_TYPE_AXIS_STATE);
	ctrpos = ctrvel = feedpos = feedvel = 0;
}
