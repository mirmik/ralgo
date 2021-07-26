#include <doctest/doctest.h>
#include <ralgo/heimer2/axisctr.h>
#include <ralgo/heimer2/command.h>

static int a = 0;

static inline
void finish_handler(void * arg, struct axis_controller * ctr)
{
	++a;
}

TEST_CASE("axisctr_concurent")
{
	heimer_reinit();

	int sts;
	double tgt;

	struct axis_state state;

	struct axis_settings settings0;
	struct axis_controller axctr0;

	struct axis_settings settings1;
	struct axis_controller axctr1;

	axis_state_init(&state, "state");

	axis_controller_init(&axctr0, "axctr0", &settings0, 1);
	axis_controller_init(&axctr1, "axctr1", &settings1, 1);
	
	axis_controller_set_handlers(&axctr0, nullptr, nullptr, finish_handler);
	axis_controller_set_handlers(&axctr1, nullptr, nullptr, finish_handler);
	
	double gain = 1000; 
	axis_controller_set_gain(&axctr0, &gain);
	axis_controller_set_gain(&axctr1, &gain);

	axis_controller_set_velocity_external(&axctr0, 10);
	axis_controller_set_velocity_external(&axctr1, 10);
	
	axis_controller_set_accdcc_external(&axctr0, 5, 5);
	axis_controller_set_accdcc_external(&axctr1, 5, 5);
	
	double forw = 100, back = -100;
	axis_controller_set_limits_external(&axctr0, &back, &forw);
	axis_controller_set_limits_external(&axctr1, &back, &forw);

	struct axis_state * state_ptr = &state;
	axis_controller_set_controlled(&axctr0, &state_ptr);
	axis_controller_set_controlled(&axctr1, &state_ptr);


	CHECK_EQ(axctr0.sigproc.ops->iterate_left(&axctr0.sigproc, NULL), &state.sig);
	CHECK_EQ(axctr0.sigproc.ops->iterate_left(&axctr0.sigproc, &state.sig), nullptr);




	tgt = 100;
	sts = axis_controller_incmove(&axctr0, 0, &tgt);
	CHECK_EQ(sts, 0);

	sts = axis_controller_incmove(&axctr0, 0, &tgt);
	CHECK_EQ(sts, -1);

	sts = axis_controller_incmove(&axctr1, 0, &tgt);
	CHECK_EQ(sts, -1);

	axis_controller_serve(&axctr0.sigproc, 5 * discrete_time_frequency());
	axis_controller_serve(&axctr1.sigproc, 5 * discrete_time_frequency());
	CHECK_EQ(axis_controller_ctrvel_external(&axctr0, 0), 10);
	CHECK_EQ(axis_controller_ctrpos_external(&axctr0, 0), 40);
	CHECK_EQ(axis_controller_ctrvel_external(&axctr1, 0), 10);
	CHECK_EQ(axis_controller_ctrpos_external(&axctr1, 0), 40);
	CHECK_EQ(a, 0);

	axis_controller_serve(&axctr0.sigproc, 12 * discrete_time_frequency());
	axis_controller_serve(&axctr1.sigproc, 12 * discrete_time_frequency());
	CHECK_EQ(axis_controller_ctrvel_external(&axctr0, 0), 0);
	CHECK_EQ(axis_controller_ctrpos_external(&axctr0, 0), 100);
	CHECK_EQ(axis_controller_ctrvel_external(&axctr1, 0), 0);
	CHECK_EQ(axis_controller_ctrpos_external(&axctr1, 0), 100);
	CHECK_EQ(a, 1);

	CHECK_EQ(axctr0.release_control_flag, 1);
	CHECK_EQ(axctr1.release_control_flag, 0);
	
	CHECK_EQ(state.sig.current_controller, &axctr0.sigproc);
	
	tgt = 200;
	sts = axis_controller_incmove(&axctr1, 0, &tgt);
	CHECK_EQ(sts, -1);

	sts = axis_controller_incmove(&axctr0, 0, &tgt);
	CHECK_EQ(sts, -1);

	axis_controller_serve(&axctr0.sigproc, 12.1 * discrete_time_frequency());
	axis_controller_serve(&axctr1.sigproc, 12.1 * discrete_time_frequency());

	CHECK_EQ(axctr0.release_control_flag, 0);
	CHECK_EQ(axctr1.release_control_flag, 0);

	CHECK_EQ(state.sig.current_controller, nullptr);
	
	tgt = 200;
	sts = axis_controller_incmove(&axctr1, 0, &tgt);
	CHECK_EQ(sts, 0);

	sts = axis_controller_incmove(&axctr0, 0, &tgt);
	CHECK_EQ(sts, -1);

/*	axis_controller_serve(&axctr0.sigproc, 5 * discrete_time_frequency());
	axis_controller_serve(&axctr1.sigproc, 5 * discrete_time_frequency());
	CHECK_EQ(axis_controller_ctrvel_external(&axctr0, 0), 10);
	CHECK_EQ(axis_controller_ctrpos_external(&axctr0, 0), 40);
	CHECK_EQ(axis_controller_ctrvel_external(&axctr1, 0), 10);
	CHECK_EQ(axis_controller_ctrpos_external(&axctr1, 0), 40);
	CHECK_EQ(a, 0);

	axis_controller_serve(&axctr0.sigproc, 12 * discrete_time_frequency());
	axis_controller_serve(&axctr1.sigproc, 12 * discrete_time_frequency());
	CHECK_EQ(axis_controller_ctrvel_external(&axctr0, 0), 0);
	CHECK_EQ(axis_controller_ctrpos_external(&axctr0, 0), 100);
	CHECK_EQ(axis_controller_ctrvel_external(&axctr1, 0), 0);
	CHECK_EQ(axis_controller_ctrpos_external(&axctr1, 0), 100);
	CHECK_EQ(a, 1);*/
}