#include <doctest/doctest.h>
#include <ralgo/heimer/axisctr.h>
#include <ralgo/heimer/command.h>

static int a = 0;

using namespace heimer;

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

	state.init("state");

	axctr0.init("axctr0", &settings0, 1);
	axctr1.init("axctr1", &settings1, 1);
	
	axctr0.set_handlers(nullptr, nullptr, finish_handler);
	axctr1.set_handlers(nullptr, nullptr, finish_handler);
	
	double gain = 1000; 
	axctr0.set_gain(&gain);
	axctr1.set_gain(&gain);

	axctr0.set_velocity_external(10);
	axctr1.set_velocity_external(10);
	
	axctr0.set_accdcc_external(5, 5);
	axctr1.set_accdcc_external(5, 5);
	
	double forw = 1000, back = -1000;
	axctr0.set_limits_external(&back, &forw);
	axctr1.set_limits_external(&back, &forw);

	struct axis_state * state_ptr = &state;
	axctr0.set_controlled(&state_ptr);
	axctr1.set_controlled(&state_ptr);

	CHECK_EQ(axctr0.iterate_left(NULL), &state);
	CHECK_EQ(axctr0.iterate_left(&state), nullptr);

	tgt = 100;
	sts = axctr0.incmove(0, &tgt);
	CHECK_EQ(sts, 0);

	sts = axctr0.incmove(0, &tgt);
	CHECK_EQ(sts, -1);

	sts = axctr1.incmove(0, &tgt);
	CHECK_EQ(sts, -1);

	axctr0.serve(5 * discrete_time_frequency());
	axctr1.serve(5 * discrete_time_frequency());
	CHECK_EQ(axctr0.ctrvel_external(0), 10);
	CHECK_EQ(axctr0.ctrpos_external(0), 40);
	CHECK_EQ(axctr1.ctrvel_external(0), 10);
	CHECK_EQ(axctr1.ctrpos_external(0), 40);
	CHECK_EQ(a, 0);

	axctr0.serve(12 * discrete_time_frequency());
	axctr1.serve(12 * discrete_time_frequency());
	CHECK_EQ(axctr0.ctrvel_external(0), 0);
	CHECK_EQ(axctr0.ctrpos_external(0), 100);
	CHECK_EQ(axctr1.ctrvel_external(0), 0);
	CHECK_EQ(axctr1.ctrpos_external(0), 100);
	CHECK_EQ(a, 1);

	CHECK_EQ(axctr0.release_control_flag, 1);
	CHECK_EQ(axctr1.release_control_flag, 0);
	
	CHECK_EQ(state.current_controller, &axctr0);
	
	tgt = 200;
	sts = axctr1.incmove(12.1 * discrete_time_frequency(), &tgt);
	CHECK_EQ(sts, -1);

	sts = axctr0.incmove(12.1 * discrete_time_frequency(), &tgt);
	CHECK_EQ(sts, -1);

	axctr0.serve(12.1 * discrete_time_frequency());
	axctr1.serve(12.1 * discrete_time_frequency());

	CHECK_EQ(axctr0.release_control_flag, 0);
	CHECK_EQ(axctr1.release_control_flag, 0);

	CHECK_EQ(state.current_controller, nullptr);
	
	tgt = 200;
	sts = axctr1.absmove(12.1 * discrete_time_frequency(), &tgt);
	CHECK_EQ(sts, 0);

	sts = axctr0.incmove(12.1 * discrete_time_frequency(), &tgt);
	CHECK_EQ(sts, -1);

	CHECK_EQ(axctr1.ctrpos_external(0), 100);

	axctr0.serve(12.1 * discrete_time_frequency());
	axctr1.serve(12.1 * discrete_time_frequency());
	CHECK_EQ(axctr0.ctrvel_external(0), 0);
	CHECK_EQ(axctr1.ctrvel_external(0), 0);
	CHECK_EQ(axctr0.ctrpos_external(0), 100);
	CHECK_EQ(a, 1);


	axctr0.serve(15.1 * discrete_time_frequency());
	axctr1.serve(15.1 * discrete_time_frequency());
	CHECK_EQ(axctr0.ctrvel_external(0), 10);
	CHECK_EQ(axctr1.ctrvel_external(0), 10);
	CHECK_EQ(axctr0.ctrpos_external(0), 120);
	CHECK_EQ(a, 1);

	axctr0.serve(24.1 * discrete_time_frequency());
	axctr1.serve(24.1 * discrete_time_frequency());
	CHECK_EQ(axctr0.ctrvel_external(0), 0);
	CHECK_EQ(axctr1.ctrvel_external(0), 0);
	CHECK_EQ(axctr0.ctrpos_external(0), 200);
	CHECK_EQ(a, 2);
}