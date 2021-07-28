#include <ralgo/heimer/axis_state.h>
#include <stdlib.h>
#include <stdio.h>

#include <ralgo/heimer/sigtypes.h>

int heimer::axis_state::info(char * data, int maxsize) 
{
	snprintf(data, maxsize, "(ctrpos:%f,ctrvel:%f,feedpos:%f,feedspd:%f)", 
		ctrpos, ctrvel, feedpos, feedvel);
	return 0;
}

void heimer::axis_state::init(const char * name) 
{
	signal_head::init(name, SIGNAL_TYPE_AXIS_STATE);
	ctrpos = ctrvel = feedpos = feedvel = 0;
}

heimer::axis_state * create_axis_state(const char * name) 
{
	heimer::axis_state * ptr = new heimer::axis_state;
	ptr->init(name);
	return ptr;
}