#include <doctest/doctest.h>
#include <ralgo/trajectory/linetraj.h>

TEST_CASE("line trajectory") 
{
	struct line_trajectory ltraj;
	struct control_phase phase;

	int64_t spos = 300;
	int64_t fpos = 400;

	line_trajectory_init_nominal_speed(
		&ltraj,
		1,
		0,
		100,
		&spos,
		&fpos,
		10,
		10,
		false
	);

	line_trajectory_attime((void*)&ltraj, 0, &phase);
	CHECK_EQ(phase.pos, 300);
	CHECK_EQ(phase.spd, 0);

	line_trajectory_attime((void*)&ltraj, 120, &phase);
	CHECK_EQ(phase.pos, 400);
	CHECK_EQ(phase.spd, 0);

	line_trajectory_attime((void*)&ltraj, 55, &phase);
	CHECK_EQ(phase.pos, 350);
	CHECK_EQ(phase.spd, 1);
}