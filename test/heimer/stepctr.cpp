#include <doctest/doctest.h>
#include <ralgo/robo/stepper_controller.h>
#include <ralgo/heimer/stepctr_applier.h>
#include <ralgo/heimer/axisctr.h>
#include <ralgo/heimer/command.h>

using namespace robo;
using namespace heimer;

TEST_CASE("stepctr")
{
	int sts;
	stepper stepsim;
	stepper_controller stepctr(&stepsim);

	stepctr.set_units_in_step(2000);

	CHECK_EQ(stepsim.steps_count(), 0);

	sts = stepctr.shift(1250);
	CHECK_EQ(sts, 0);
	CHECK_EQ(stepctr.virtual_pos(), 1250);
	CHECK_EQ(stepsim.steps_count(), 0);

	stepctr.shift(1250);
	CHECK_EQ(stepsim.steps_count(), 1);

	stepctr.shift(-1250);
	CHECK_EQ(stepsim.steps_count(), 1);

	stepctr.shift(-1250);
	CHECK_EQ(stepsim.steps_count(), 0);
}

TEST_CASE("fixed_frequency_stepctr")
{
	stepper stepsim;
	fixed_frequency_stepper_controller stepctr(&stepsim);

	stepctr.set_units_in_step(10000);

	stepctr.set_frequency(0.1);

	stepctr.set_speed(7 * 1000);
	for (int i = 0; i < 10000; ++i)
	{
		stepctr.constant_frequency_serve();
	}
	CHECK_EQ(stepsim.steps_count(), 7000);

	stepctr.set_speed(-7 * 1000);
	for (int i = 0; i < 10000; ++i)
	{
		stepctr.constant_frequency_serve();
	}
	CHECK_EQ(stepsim.steps_count(), 0);
}


TEST_CASE("stepctr_applier")
{
	int sts;
	heimer::heimer_reinit();
	dprln("stepctr_applier");

	stepper stepsim;
	fixed_frequency_stepper_controller stepctr(&stepsim);
	stepctr.set_frequency(0.1);

	axis_state x("x");
	axis_controller xctr("xctr", { &x });

	stepctr_applier applier("xapply", &stepctr, &x);

	stepctr.set_units_in_step(10000);
	stepctr.set_frequency(0.01);

	applier.set_gain(4194304);
	//applier.set_compkoeff(0.00001);

	xctr.set_velocity_external(1);
	xctr.set_acceleration_external(2);
	xctr.set_decceleration_external(2);
	double forw = 100, back = -100;
	xctr.set_limits_external(&back, &forw);

	applier.feedback(0);
	xctr.feedback(0);

	CHECK_EQ(x.feedpos, 0);
	CHECK_EQ(x.feedvel, 0);

	CHECK_EQ(xctr.external_velocity(), 1);
	sts = xctr.absmove(0, {10.});
	CHECK_EQ(xctr.is_active(), true);
	CHECK_EQ(sts, 0);

	sts = applier.feedback(1);
	CHECK_EQ(sts, 0);
	sts = xctr.feedback(1);
	CHECK_EQ(sts, 0);
	sts = xctr.serve(1);
	CHECK_EQ(xctr.is_active(), true);
	CHECK_EQ(sts, 0);
	sts = applier.serve(1);
	CHECK_EQ(sts, 0);

	CHECK_NE(x.ctrvel, 0);


	for (int sec = 0; sec < 2; ++sec)
	{
		for (int i = 1; i < 100; ++i)
		{
			int curtime = sec * 1000 + i * 10;
			for (int j = 0; j < 100; ++j)
			{
				sts = stepctr.constant_frequency_serve();
				CHECK_EQ(sts, 0);
			}
			applier.feedback(curtime);
			xctr.feedback(curtime);
			xctr.serve(curtime);
			applier.serve(curtime);
		}
	}

	CHECK_NE(x.ctrpos, 0);
	CHECK_EQ(stepsim.steps_count(), 4194304);
}
