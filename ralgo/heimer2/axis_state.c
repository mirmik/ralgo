#include <ralgo/heimer2/axis_state.h>
#include <stdlib.h>
#include <stdio.h>

#include <ralgo/heimer2/sigtypes.h>

int axis_state_info(struct signal_head * sig, char * data, int maxsize) 
{
	struct axis_state * s = mcast_out(sig, struct axis_state, sig);
	snprintf(data, maxsize, "(ctrpos:%ld,ctrvel:%f,feedpos:%ld,feedspd:%f)", 
		s->ctrpos, s->ctrvel, s->feedpos, s->feedvel);
	return 0;
};

const struct signal_head_operations axis_state_ops = {
	.info = axis_state_info
};

void axis_state_init(struct axis_state * state, const char * name) 
{
	signal_head_init(&state->sig, name, SIGNAL_TYPE_AXIS_STATE, &axis_state_ops);
	state->ctrpos = state->ctrvel = state->feedpos = state->feedvel = 0;
}

struct axis_state * create_axis_state(const char * name) 
{
	struct axis_state * ptr = (struct axis_state *) malloc(sizeof(struct axis_state));
	axis_state_init(ptr, name);
	return ptr;
}