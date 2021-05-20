#include <doctest/doctest.h>
#include <ralgo/madgwick.h>
#include <nos/print.h>

TEST_CASE("madgwick") 
{
	ralgo::madgwick madgwick;

	madgwick.setKoeff(100, 0.1);

	for (int i = 0; i < 2000; ++i)
	madgwick.update(
		0,0,0,
		0,0,-1,
		0,1,0
	);
	nos::println(madgwick.quat());
	nos::println(madgwick.hx, madgwick.hy);
	CHECK_EQ(madgwick.quat()[3], doctest::Approx(0.70710).epsilon(0.001));
	CHECK_EQ(madgwick.quat()[2], doctest::Approx(-0.70710).epsilon(0.001));

	for (int i = 0; i < 200000; ++i) {
	madgwick.update(
		0,0,0,
		0,0,-1,
		1,0,0
	);
	nos::println(madgwick.quat());
	nos::println(madgwick.hx, madgwick.hy);
	}
	nos::println(madgwick.hx, madgwick.hy);
	CHECK_EQ(madgwick.quat()[3], doctest::Approx(1).epsilon(0.001));
	CHECK_EQ(madgwick.quat()[2], doctest::Approx(0).epsilon(0.001));

	for (int i = 0; i < 2000; ++i)
	madgwick.update(
		0,0,0,
		0,0,-1,
		0,-1,0
	);
	nos::println(madgwick.quat());
	nos::println(madgwick.hx, madgwick.hy);
	CHECK_EQ(madgwick.quat()[3], doctest::Approx(0.70710).epsilon(0.001));
	CHECK_EQ(madgwick.quat()[2], doctest::Approx(0.70710).epsilon(0.001));

	for (int i = 0; i < 2000; ++i)
	madgwick.update(
		0,0,0,
		0,0,-1,
		-1,0,0
	);
	nos::println(madgwick.quat());
	nos::println(madgwick.hx, madgwick.hy);
	CHECK_EQ(madgwick.quat()[3], doctest::Approx(0).epsilon(0.001));
	CHECK_EQ(madgwick.quat()[2], doctest::Approx(1).epsilon(0.001));
}