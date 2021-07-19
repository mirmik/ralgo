#include <doctest/doctest.h>
#include <ralgo/heimer2/axisctr.h>

TEST_CASE("axisctr") 
{
	struct axis_state state;
	struct axis_controller axctr;

	axis_controller_set_gain(&axctr, 1000);
	axis_controller_set_limits_external(&axctr, -100, 100);
}