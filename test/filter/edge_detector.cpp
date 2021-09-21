#include <ralgo/filter/edge_detector.h>
#include <doctest/doctest.h>

#include <ralgo/linalg/vecops.h>

TEST_CASE("edge_detector") 
{
	ralgo::rising_edge_detector detector(0.5);
	int acc = 0;

	auto time = ralgo::vecops::linspace(0., 10.*2*M_PI, 1000);
	auto signal = ralgo::vecops::sin(time);

	for (auto & val : signal) 
	{
		ralgo::EdgeDetectorStatus sig = detector.serve(val);
		acc += sig == ralgo::EdgeDetectorStatus::RisingEvent ? 1 : 0;
	}

	CHECK_EQ(acc, 10);
}
