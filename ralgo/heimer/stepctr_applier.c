#include <ralgo/heimer/stepctr_applier.h>
#include <igris/math.h>

const struct signal_processor_operations stepctr_applier_ops =
{
	.serve = stepctr_applier_serve
};

void stepctr_applier_init(
    struct stepctr_applier * applier,
    const char * name,
    struct stepctr_controller * stepctr,
    struct axis_state * state
)
{
	signal_processor_init(&applier->sigproc, name, &stepctr_applier_ops);
	applier->controlled_stepctr = stepctr;
	applier->state = state;

	signal_head_get(&state->sig);
}

void stepctr_applier_deinit(struct stepctr_applier * applier)
{
	signal_processor_deinit(&applier->sigproc);
	signal_head_put(&applier->state->sig);
}

void stepctr_applier_serve(struct signal_processor * proc, disctime_t time)
{
	struct stepctr_applier * applier = mcast_out(proc, struct stepctr_applier, sigproc);
	int64_t errpos = applier->state->ctrpos - applier->state->feedpos;

	if (ABS(errpos) > applier->deviation_error_limit)
	{
		stepctr_controller_set_speed(applier->controlled_stepctr, 0);

		//char str[56];
		//sprintf(str, "position deviation error : mnemo:%s", parent::mnemo());
		//ralgo::warn(str);

		//force_stop_interrupt_args msg("position deviation error");
		//parent::throw_interrupt(&msg);
	}

	// Скорость вычисляется как
	// сумма уставной скорости на
	float compspd = applier->state->ctrvel + applier->compkoeff * errpos;
	stepctr_controller_set_speed(applier->controlled_stepctr, compspd);

}

