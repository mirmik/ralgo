#include <doctest/doctest.h>
#include <ralgo/robo/quadgen4_arduino.h>

TEST_CASE("quadgen4_arduino") 
{
	struct quadgen4_arduino quadgen;

	quadgen4_arduino_init(&quadgen, 8,10,9,11);
	quadgen4_arduino_set(&quadgen, 3);
}