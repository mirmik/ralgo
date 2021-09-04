#include <ralgo/cnc/planner.h>
#include <doctest/doctest.h>

TEST_CASE("planblock")
{
	cnc::planner_block block;

	int64_t task[] = { 10, 20 }; 
	double mults[] = { 1, 1 };
	block.set_state(task, 2, 0.5, 0.25, mults);
}