#include <doctest/doctest.h>
#include <ralgo/calibration/magnetometer.h>

TEST_CASE("magnetometer") 
{
	int yaw_total = 6;
	int pitch_total = 6; 

	linalg<float,3> array [yaw_total * (pitch_total - 2) +2];

	auto collector = spherical_cloud_collector(yaw_total, pitch_total, array);


	CHECK_EQ(collector.yaw_by_index(1), M_PI / 3);
}