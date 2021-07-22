#include <doctest/doctest.h>
#include <ralgo/heimer2/axstate_linear_processor.h>

TEST_CASE("axstate_linear_processor") 
{
	struct axis_state a, b, c, d;
	struct axis_state * left[] = { &a, &b };
	struct axis_state * right[] = { &c, &d };

	struct axstate_linear_processor linproc;

	float matrix[] = { 2, 0.5, 0, 4 }; 
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

	CHECK_EQ(inverse_matrix[0], 0.5);
	CHECK_EQ(inverse_matrix[1], -0.0625);
	CHECK_EQ(inverse_matrix[2], 0);
	CHECK_EQ(inverse_matrix[3], 0.25);
}