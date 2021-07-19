#include <doctest/doctest.h>
#include <ralgo/heimer2/axisctr.h>

TEST_CASE("axisctr") 
{
	struct axis_state state;
	struct axis_controller axctr;

	axis_controller_set_gain(&axctr, 1000);
	axis_controller_set_velocity_external(&axctr, 10);
	axis_controller_set_accdcc_external(&axctr, 4, 4);
	axis_controller_set_limits_external(&axctr, -100, 100);

	axis_controller_set_controlled(&axctr, &state);

	axis_controller_serve(&axctr, 0);
}