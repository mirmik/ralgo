#include <doctest/doctest.h>
#include <ralgo/heimer/axstate_pose3_chain_controller.h>

TEST_CASE("axstate_pose3_controller") 
{
	heimer::axstate_pose3_chain_controller posectr("posectr", 6);	
}