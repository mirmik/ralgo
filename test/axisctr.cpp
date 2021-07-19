#include <doctest/doctest.h>
#include <ralgo/heimer2/axisctr.h>

#include <nos/print.h>

static int a = 0;

void finish_handler(void * arg, struct axis_controller * ctr)
{
	++a;
}

TEST_CASE("axisctr")
{
	int sts;

	struct axis_state state;
	struct axis_controller axctr;

	axis_controller_init(&axctr);
	axis_controller_set_handlers(&axctr, nullptr, nullptr, finish_handler);
	axis_controller_set_gain(&axctr, 1000);
	axis_controller_set_velocity_external(&axctr, 10);
	axis_controller_set_accdcc_external(&axctr, 5, 5);
	axis_controller_set_limits_external(&axctr, -100, 100);

	axis_controller_set_controlled(&axctr, &state);
	axis_controller_incmove(&axctr, 0, 100);

	CHECK_EQ(axctr.vel, 10.f * DISTANCE_MULTIPLIER / discrete_time_frequency() * 1000);
	CHECK_EQ(axctr.acc, 5.f * DISTANCE_MULTIPLIER / discrete_time_frequency() / discrete_time_frequency() * 1000);
	CHECK_EQ(axctr.dcc, 5.f * DISTANCE_MULTIPLIER / discrete_time_frequency() / discrete_time_frequency() * 1000);

	CHECK_EQ(axctr.lintraj.ftim, 10 * discrete_time_frequency());

	axis_controller_serve(&axctr, 0);
	CHECK_EQ(axis_controller_ctrvel_external(&axctr), 0);
	CHECK_EQ(axis_controller_ctrpos_external(&axctr), 0);
	CHECK_EQ(a, 0);

	axis_controller_serve(&axctr, 1 * discrete_time_frequency());
	CHECK_EQ(axis_controller_ctrvel_external(&axctr), 5);
	CHECK_EQ(a, 0);

	axis_controller_serve(&axctr, 2 * discrete_time_frequency());
	CHECK_EQ(axis_controller_ctrvel_external(&axctr), 10);
	CHECK_EQ(a, 0);

	axis_controller_serve(&axctr, 5 * discrete_time_frequency());
	CHECK_EQ(axis_controller_ctrvel_external(&axctr), 10);
	CHECK_EQ(axis_controller_ctrpos_external(&axctr), 40);
	CHECK_EQ(a, 0);

	axis_controller_serve(&axctr, 6 * discrete_time_frequency());
	CHECK_EQ(axis_controller_ctrvel_external(&axctr), 10);
	CHECK_EQ(axis_controller_ctrpos_external(&axctr), doctest::Approx(50));
	CHECK_EQ(a, 0);

	axis_controller_serve(&axctr, 10 * discrete_time_frequency());
	CHECK_EQ(axis_controller_ctrvel_external(&axctr), doctest::Approx(10));
	CHECK_EQ(axis_controller_ctrpos_external(&axctr), doctest::Approx(90));
	CHECK_EQ(a, 0);

	axis_controller_serve(&axctr, 11 * discrete_time_frequency());
	CHECK_EQ(axis_controller_ctrvel_external(&axctr), doctest::Approx(5));
	CHECK_EQ(axis_controller_ctrpos_external(&axctr), doctest::Approx(97.5));
	CHECK_EQ(a, 0);

	axis_controller_serve(&axctr, 12 * discrete_time_frequency());
	CHECK_EQ(axis_controller_ctrvel_external(&axctr), doctest::Approx(0));
	CHECK_EQ(axis_controller_ctrpos_external(&axctr), doctest::Approx(100));
	CHECK_EQ(a, 1);
}