#include <ralgo/cnc/interpreter.h>
#include <doctest/doctest.h>

TEST_CASE("interpreter") 
{
	igris::ring<cnc::planner_block, 10> blocks_ring;

	cnc::interpreter interpreter(&blocks_ring);
}
