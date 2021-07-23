#include <doctest/doctest.h>
#include <ralgo/heimer2/axstate_linear_processor.h>

TEST_CASE("axstate_linear_processor") 
{
	struct axis_state a, b, c, d;
	struct axis_state * left[] = { &a, &b };
	struct axis_state * right[] = { &c, &d };

	axis_state_init(&a, "a");
	axis_state_init(&b, "b");
	axis_state_init(&c, "c");
	axis_state_init(&d, "d");

	struct axstate_linear_processor linproc;

	float matrix[] = { 2, 0.5, 0, 1 }; 
	float inverse_matrix[4];

	axstate_linear_processor_init(
		&linproc,
		"linproc",
		2,
		left,
		right,
		matrix,
		inverse_matrix
	);
	axstate_linear_processor_evaluate_invertion(&linproc);

	c.ctrpos = 200;
	d.ctrpos = 100;
	c.ctrvel = 20;
	d.ctrvel = 10;

	signal_processor_serve(&linproc.proc, 0);

	CHECK_EQ(a.ctrpos, 450);
	CHECK_EQ(b.ctrpos, 100);
	CHECK_EQ(a.ctrvel, 45);
	CHECK_EQ(b.ctrvel, 10);

	a.feedpos = 200;
	b.feedpos = 100;
	a.feedvel = 20;
	b.feedvel = 10;

	signal_processor_feedback(&linproc.proc, 0);

	CHECK_EQ(c.feedpos, 0);
	CHECK_EQ(d.feedpos, 0);
	CHECK_EQ(c.feedvel, 0);
	CHECK_EQ(d.feedvel, 0);


	signal_head_deinit(&a.sig);
	signal_head_deinit(&b.sig);
	signal_head_deinit(&c.sig);
	signal_head_deinit(&d.sig);
}