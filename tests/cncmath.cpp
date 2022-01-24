#include <doctest/doctest.h>
#include <ralgo/cnc/math.h>

TEST_CASE("cnc.math") 
{
	ralgo::vector<double> arg{0,11,0};
	ralgo::vector<double> dir = ralgo::vecops::normalize<ralgo::vector<double>>(arg);
	CHECK_EQ(dir, ralgo::vector<double>{0,1,0});
}