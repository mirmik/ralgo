#include <doctest/doctest.h>
#include <ralgo/heimer2/axisctr.h>
#include <ralgo/heimer2/command.h>

#include <nos/print.h>

static int a = 0;

static inline
void finish_handler(void * arg, struct axis_controller * ctr)
{
	++a;
}

TEST_CASE("axisctr")
{
	heimer_reinit();

	int sts;

	struct axis_state state;
	struct axis_settings settings;
	struct axis_controller axctr;

	axis_state_init(&state, "state");

	axis_controller_init(&axctr, "axctr", &settings, 1);
	axis_controller_set_handlers(&axctr, nullptr, nullptr, finish_handler);
	double gain = 1000; axis_controller_set_gain(&axctr, &gain);
	axis_controller_set_velocity_external(&axctr, 10);
	axis_controller_set_accdcc_external(&axctr, 5, 5);
	double forw = 100, back = -100;
	axis_controller_set_limits_external(&axctr, &back, &forw);

	struct axis_state * state_ptr = &state;
	axis_controller_set_controlled(&axctr, &state_ptr);
	double tgt = 100;
	sts = axis_controller_incmove(&axctr, 0, &tgt);
	CHECK_EQ(sts, 0);

	CHECK_EQ(axctr.vel, doctest::Approx(10.f / discrete_time_frequency()));
	CHECK_EQ(axctr.acc, doctest::Approx(5.f / discrete_time_frequency() / discrete_time_frequency()));
	CHECK_EQ(axctr.dcc, doctest::Approx(5.f / discrete_time_frequency() / discrete_time_frequency()));

	CHECK_EQ(axctr.lintraj.ftim, 10 * discrete_time_frequency());

	axis_controller_serve(&axctr.sigproc, 0);
	CHECK_EQ(axis_controller_ctrvel_external(&axctr, 0), 0);
	CHECK_EQ(axis_controller_ctrpos_external(&axctr, 0), 0);
	CHECK_EQ(a, 0);

	axis_controller_serve(&axctr.sigproc, 1 * discrete_time_frequency());
	CHECK_EQ(axis_controller_ctrvel_external(&axctr, 0), 5);
	CHECK_EQ(a, 0);

	axis_controller_serve(&axctr.sigproc, 2 * discrete_time_frequency());
	CHECK_EQ(axis_controller_ctrvel_external(&axctr, 0), 10);
	CHECK_EQ(a, 0);

	axis_controller_serve(&axctr.sigproc, 5 * discrete_time_frequency());
	CHECK_EQ(axis_controller_ctrvel_external(&axctr, 0), 10);
	CHECK_EQ(axis_controller_ctrpos_external(&axctr, 0), 40);
	CHECK_EQ(a, 0);

	axis_controller_serve(&axctr.sigproc, 6 * discrete_time_frequency());
	CHECK_EQ(axis_controller_ctrvel_external(&axctr, 0), 10);
	CHECK_EQ(axis_controller_ctrpos_external(&axctr, 0), doctest::Approx(50));
	CHECK_EQ(a, 0);

	axis_controller_serve(&axctr.sigproc, 10 * discrete_time_frequency());
	CHECK_EQ(axis_controller_ctrvel_external(&axctr, 0), doctest::Approx(10));
	CHECK_EQ(axis_controller_ctrpos_external(&axctr, 0), doctest::Approx(90));
	CHECK_EQ(a, 0);

	axis_controller_serve(&axctr.sigproc, 11 * discrete_time_frequency());
	CHECK_EQ(axis_controller_ctrvel_external(&axctr, 0), doctest::Approx(5));
	CHECK_EQ(axis_controller_ctrpos_external(&axctr, 0), doctest::Approx(97.5));
	CHECK_EQ(a, 0);

	signal_processor_serve(&axctr.sigproc, 12 * discrete_time_frequency());
	CHECK_EQ(axis_controller_ctrvel_external(&axctr, 0), doctest::Approx(0));
	CHECK_EQ(axis_controller_ctrpos_external(&axctr, 0), doctest::Approx(100));
	CHECK_EQ(a, 1);
}



TEST_CASE("axisctr_multiax")
{
	heimer_reinit();
	a=0;

	int sts;

	struct axis_state state0;
	struct axis_state state1;
	struct axis_settings settings[2];
	struct axis_controller axctr;

	axis_state_init(&state0, "state0");
	axis_state_init(&state1, "state1");

	axis_controller_init(&axctr, "axctr", settings, 2);
	axis_controller_set_handlers(&axctr, nullptr, nullptr, finish_handler);
	double gain = 1000; axis_controller_set_gain(&axctr, &gain);
	axis_controller_set_velocity_external(&axctr, 10);
	axis_controller_set_accdcc_external(&axctr, 5, 5);
	
	double forw[2] = { 100,  100 }; 
	double back[2] = {-100, -100 };
	axis_controller_set_limits_external(&axctr, back, forw);

	struct axis_state * states[] = { &state0, &state1 };
	axis_controller_set_controlled(&axctr, states);
	
	double tgt[] = { 100, 100 };
	sts = axis_controller_incmove(&axctr, 0, tgt);
	CHECK_EQ(sts, 0);

	CHECK_EQ(axctr.vel, doctest::Approx(10.f / discrete_time_frequency()));
	CHECK_EQ(axctr.acc, doctest::Approx(5.f / discrete_time_frequency() / discrete_time_frequency()));
	CHECK_EQ(axctr.dcc, doctest::Approx(5.f / discrete_time_frequency() / discrete_time_frequency()));

	CHECK_EQ(axctr.lintraj.ftim, doctest::Approx(10 * sqrt(2) * discrete_time_frequency()));

	axis_controller_serve(&axctr.sigproc, 0);
	CHECK_EQ(axis_controller_ctrvel_external(&axctr, 0), 0);
	CHECK_EQ(axis_controller_ctrpos_external(&axctr, 0), 0);
	CHECK_EQ(a, 0);

	axis_controller_serve(&axctr.sigproc, 1 * discrete_time_frequency());
	CHECK_EQ(axis_controller_ctrvel_external(&axctr, 0), doctest::Approx(5 / sqrt(2)));
	CHECK_EQ(a, 0);

	axis_controller_serve(&axctr.sigproc, 2 * discrete_time_frequency());
	CHECK_EQ(axis_controller_ctrvel_external(&axctr, 0), doctest::Approx(10 / sqrt(2)));
	CHECK_EQ(a, 0);

	axis_controller_serve(&axctr.sigproc, 5 * discrete_time_frequency());
	CHECK_EQ(axis_controller_ctrvel_external(&axctr, 0), doctest::Approx(10 / sqrt(2)));
	CHECK_EQ(axis_controller_ctrpos_external(&axctr, 0), doctest::Approx(40 / sqrt(2)));
	CHECK_EQ(a, 0);

	axis_controller_serve(&axctr.sigproc, 6 * discrete_time_frequency());
	CHECK_EQ(axis_controller_ctrvel_external(&axctr, 0), doctest::Approx(10 / sqrt(2)));
	CHECK_EQ(axis_controller_ctrpos_external(&axctr, 0), doctest::Approx(50 / sqrt(2)));
	CHECK_EQ(a, 0);

	axis_controller_serve(&axctr.sigproc, 10 * discrete_time_frequency());
	CHECK_EQ(axis_controller_ctrvel_external(&axctr, 0), doctest::Approx(10 / sqrt(2)));
	CHECK_EQ(axis_controller_ctrpos_external(&axctr, 0), doctest::Approx(90 / sqrt(2)));
	CHECK_EQ(a, 0);

	axis_controller_serve(&axctr.sigproc, (16.142 - 1) * discrete_time_frequency());
	CHECK_EQ(axis_controller_ctrvel_external(&axctr, 0), doctest::Approx(5  / sqrt(2)));
	CHECK_EQ(a, 0);

	signal_processor_serve(&axctr.sigproc, 16.142 * discrete_time_frequency());
	CHECK_EQ(axis_controller_ctrvel_external(&axctr, 0), doctest::Approx(0));
	CHECK_EQ(axis_controller_ctrpos_external(&axctr, 0), doctest::Approx(100));
	CHECK_EQ(a, 1);
}