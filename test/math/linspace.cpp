#include <doctest/doctest.h>
#include <ralgo/linspace.h>

#include <nos/print.h>
#include <vector>

TEST_CASE("linspace") 
{
	CHECK_EQ(ralgo::lerp(1.,4.,0.5), 2.5);
	CHECK_EQ(ralgo::lerp(1.,4.,0.), 1);
	CHECK_EQ(ralgo::lerp(1.,4.,1.f), 4);

	auto ls = ralgo::linspace<double>(1,4,4);
	std::vector<double> vec(ls.begin(), ls.end());
	CHECK_EQ(ls[0], doctest::Approx(1));
	CHECK_EQ(ls[1], doctest::Approx(2));
	CHECK_EQ(ls[2], doctest::Approx(3));
	CHECK_EQ(ls[3], doctest::Approx(4));

	auto ls2 = ralgo::linspace<double>(1,6,5,false);
	std::vector<double> vec2(ls2.begin(), ls2.end());
	CHECK_EQ(ls2[0], doctest::Approx(1));
	CHECK_EQ(ls2[1], doctest::Approx(2));
	CHECK_EQ(ls2[2], doctest::Approx(3));
	CHECK_EQ(ls2[3], doctest::Approx(4));
	CHECK_EQ(ls2[4], doctest::Approx(5));
}