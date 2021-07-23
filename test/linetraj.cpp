#include <doctest/doctest.h>
#include <ralgo/trajectory/linetraj.h>

TEST_CASE("line trajectory") 
{
	struct line_trajectory ltraj;
	position_t pos;
	velocity_t   vel;

	position_t spos_buffer;
	position_t fpos_buffer;
	position_t spos = 300;
	position_t fpos = 400;

	line_trajectory_init(&ltraj, 1, &spos, &fpos);

	line_trajectory_init_nominal_speed(
		&ltraj,
		0,
		100,
		&spos,
		&fpos,
		10,
		10,
		false
	);

	line_trajectory_attime((void*)&ltraj, 0, &pos, &vel);
	CHECK_EQ(pos, 300);
	CHECK_EQ(vel, 0);

	line_trajectory_attime((void*)&ltraj, 120, &pos, &vel);
	CHECK_EQ(pos, 400);
	CHECK_EQ(vel, 0);

	line_trajectory_attime((void*)&ltraj, 55, &pos, &vel);
	CHECK_EQ(pos, 350);
	CHECK_EQ(vel, 1);
}