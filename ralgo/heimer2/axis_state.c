#include <ralgo/heimer2/axis_state.h>
#include <stdlib.h>

void axis_state_init(struct axis_state * state, const char * name) 
{
	signal_head_init(&state->sig, name);
}

struct axis_state * create_axis_state(const char * name) 
{
	struct axis_state * ptr = (struct axis_state *) malloc(sizeof(struct axis_state));
	axis_state_init(ptr, name);
	return ptr;
}