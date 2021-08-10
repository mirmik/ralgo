#include <doctest/doctest.h>
#include <ralgo/robo/stepper_controller.h>
#include <ralgo/heimer/stepctr_applier.h>

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