#include <doctest/doctest.h>
#include <ralgo/heimer/command.h>
#include <ralgo/heimer/axstate_linear_processor.h>

TEST_CASE("axstate_linear_processor") 
{
	heimer_reinit();

	axis_state a, b, c, d;
	axis_state * left[] = { &a, &b };
	axis_state * right[] = { &c, &d };

	a.init("a");
	b.init("b");
	c.init("c");
	d.init("d");

	axstate_linear_processor linproc;

	float matrix[] = { 2, 0.5, 0, 1 }; 
	float inverse_matrix[4];

	linproc.init(
		"linproc",
		2,
		left,
		right,
		matrix,
		inverse_matrix
	);
	linproc.evaluate_invertion();
	CHECK_NE(inverse_matrix[0], 0);

	c.ctrpos = 200;
	d.ctrpos = 100;
	c.ctrvel = 20;
	d.ctrvel = 10;

	linproc.serve(0);

	CHECK_EQ(a.ctrpos, 450);
	CHECK_EQ(b.ctrpos, 100);
	CHECK_EQ(a.ctrvel, 45);
	CHECK_EQ(b.ctrvel, 10);

	a.feedpos = a.ctrpos;
	b.feedpos = b.ctrpos;
	a.feedvel = a.ctrvel;
	b.feedvel = b.ctrvel;

	linproc.feedback(0);

	CHECK_EQ(c.feedpos, 200);
	CHECK_EQ(d.feedpos, 100);
	CHECK_EQ(c.feedvel, 20);
	CHECK_EQ(d.feedvel, 10);

	a.deinit();
	b.deinit();
	c.deinit();
	d.deinit();
}