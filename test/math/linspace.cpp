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

	nos::println(vec);
	CHECK_EQ(vec, std::vector<double>{1,2,3,4});
}