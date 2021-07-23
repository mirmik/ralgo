#include <doctest/doctest.h>
#include <ralgo/heimer2/distance.h>

TEST_CASE("heimer_distance") 
{
	CHECK_EQ(heimdist(10), 10 << 24);
	CHECK_EQ(heimpos_cos(heimdeg(45.f)), doctest::Approx(float(1./sqrt(2.))));
}
