#include <ralgo/geom/zone_check.h>
#include <linalg/linalg.h>
#include <main.h>

using namespace ralgo;

LT_BEGIN_TEST(ralgo_test_suite, zone_check)
{
	linalg::vec<double,2> arr[] = {
		{-1,-1},
		{0,1},
		{1,-1}
	};

	linalg::vec<double,2> t0 = {0,0};
	linalg::vec<double,2> t1 = {1,1};

	CHECK(point2_in_polygon(arr, t0) == true);
	CHECK(point2_in_polygon(arr, t1) == false);
}
LT_END_TEST(zone_check)